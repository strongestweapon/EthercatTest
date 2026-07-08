#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TL-E 시리즈 EtherCAT 통합형 스텝모터(TLC86E 등) 테스트용 간단 GUI
- pysoem(SOEM) 기반 EtherCAT 마스터
- CiA402 PP(Profile Position) 모드로 위치 이동 테스트
- 상태워드/실제위치/실제속도/에러코드를 실시간 모니터링 (추측 금지, 실측 확인용)

원칙:
- 이 드라이버는 PP 모드에서 드라이버가 내부 궤적계산을 하므로, 맥/Free-Run에서도
  타이밍 부담이 적어 테스트에 적합하다.
- PDO 매핑은 PREOP에서 SDO로 명시적으로 구성한다(디폴트 매핑에 의존하지 않음).

실행:  sudo python3 tlc_ethercat_gui.py     (raw 소켓 접근에 관리자 권한 필요)
"""

import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

try:
    import pysoem
    PYSOEM_OK = True
    PYSOEM_ERR = ""
except Exception as e:  # noqa
    PYSOEM_OK = False
    PYSOEM_ERR = str(e)


# ---- CiA402 오브젝트 인덱스 (매뉴얼 3.4 Motion Parameters 기준) --------------
OD_CONTROLWORD   = 0x6040   # U16
OD_STATUSWORD    = 0x6041   # U16
OD_MODE          = 0x6060   # I8  (1:PP 3:PV 6:HM 8:CSP)
OD_MODE_DISPLAY  = 0x6061   # I8
OD_ACTUAL_POS    = 0x6064   # I32 pulse
OD_ACTUAL_SPEED  = 0x606C   # I32 pulse/s
OD_TARGET_POS    = 0x607A   # I32 pulse
OD_PROFILE_SPEED = 0x6081   # U32 pulse/s
OD_ACCEL         = 0x6083   # U32 pulse/s^2
OD_DECEL         = 0x6084   # U32 pulse/s^2
OD_ERROR_CODE    = 0x603F   # U16 (0:정상 0xFF01 과전류 등)
OD_BRAKE_ENABLE  = 0x2403   # U16 브레이크 출력 활성화 (0:잠금 1:자동해제)
OD_BRAKE_OPEN    = 0x2404   # U16 브레이크 해제지연 ms (기본 200)
OD_BRAKE_CLOSE   = 0x2405   # U16 브레이크 잠금지연 ms (기본 200)

# 제어워드 비트 (매뉴얼 4.1.1)
CW_ENABLE      = 0x000F   # 0x6->0x7->0xF 순으로 도달, 최종 운전허가 상태
CW_NEW_SETPT   = 0x0010   # bit4: 0->1 신규 목표위치 트리거
CW_IMMEDIATE   = 0x0020   # bit5: 즉시 갱신
CW_RELATIVE    = 0x0040   # bit6: 상대이동
CW_FAULT_RESET = 0x0080   # bit7: 알람 리셋
CW_HALT        = 0x0100   # bit8: 감속정지

MODE_PP = 1

# 에러코드 해석 (매뉴얼 3.4)
ERROR_TEXT = {
    0x0000: "정상",
    0xFF01: "과전류(Overcurrent)",
    0xFF02: "과전압(Overvoltage)",
    0xFF03: "저전압(Undervoltage)",
    0xFF04: "상 오류(Phase error)",
    0xFF05: "위치편차(Position deviation)",
}


# =============================================================================
#  EtherCAT 워커 스레드  (모든 pysoem 호출은 이 스레드에서만 수행)
# =============================================================================
class EtherCATWorker(threading.Thread):
    CYCLE = 0.004   # 4ms 주기 (Free-Run)

    def __init__(self, log_q, status_d):
        super().__init__(daemon=True)
        self.log_q = log_q
        self.status = status_d          # GUI가 폴링해서 읽는 공유 상태 dict
        self.cmd_q = queue.Queue()
        self._stopev = threading.Event()

        self.master = None
        self.slave = None
        self.op = False                 # OP 상태 여부

        # PDO로 주고받는 값 (공유)
        self.control_word = 0x0000
        self.target_pos = 0
        self.profile_speed = 10000

        # 설정값 (config 시 SDO로 기록)
        self.cfg_speed = 10000
        self.cfg_accel = 100000
        self.cfg_decel = 100000

        self._err_poll = 0

    # ---- 외부(GUI)에서 호출하는 명령 인터페이스 ----
    def post(self, *cmd):
        self.cmd_q.put(cmd)

    def shutdown(self):
        self._stopev.set()

    def log(self, msg):
        self.log_q.put(msg)

    # ---- 메인 루프 ----
    def run(self):
        while not self._stopev.is_set():
            try:
                if self.op:
                    self._cycle()
                    self._drain_cmds()
                    time.sleep(self.CYCLE)
                else:
                    # 미연결 상태: 명령 대기
                    try:
                        cmd = self.cmd_q.get(timeout=0.2)
                        self._dispatch(cmd)
                    except queue.Empty:
                        pass
            except Exception as e:  # noqa
                self.log(f"[ERR] 루프 예외: {e}")
                self._teardown()
                self.status["state"] = "오류/연결끊김"
                time.sleep(0.3)
        self._teardown()

    def _drain_cmds(self):
        try:
            while True:
                self._dispatch(self.cmd_q.get_nowait())
        except queue.Empty:
            pass

    # ---- 한 주기 PDO 교환 ----
    def _cycle(self):
        # RxPDO: [controlword U16][target_pos I32][profile_speed U32]
        self.slave.output = struct.pack("<HiI", self.control_word & 0xFFFF,
                                        self.target_pos, self.profile_speed & 0xFFFFFFFF)
        self.master.send_processdata()
        wkc = self.master.receive_processdata(2000)
        if wkc < 1:
            self.status["wkc_ok"] = False
        else:
            self.status["wkc_ok"] = True

        # TxPDO: [statusword U16][actual_pos I32][actual_speed I32]
        data = bytes(self.slave.input)
        if len(data) >= 10:
            sw, apos, aspd = struct.unpack("<Hii", data[:10])
            self.status["status_word"] = sw
            self.status["actual_pos"] = apos
            self.status["actual_speed"] = aspd

        # 에러코드는 SDO로 가끔만 읽음 (약 0.5초마다)
        self._err_poll += 1
        if self._err_poll >= 125:
            self._err_poll = 0
            try:
                raw = self.slave.sdo_read(OD_ERROR_CODE, 0)
                self.status["error_code"] = struct.unpack("<H", raw[:2])[0]
            except Exception:
                pass

    # ---- 명령 처리 ----
    def _dispatch(self, cmd):
        name = cmd[0]
        if name == "connect":
            self._connect(cmd[1])
        elif name == "disconnect":
            self._teardown()
            self.status["state"] = "연결끊김"
            self.log("[i] 연결 해제")
        elif name == "config":
            self.cfg_speed, self.cfg_accel, self.cfg_decel = cmd[1], cmd[2], cmd[3]
            self.profile_speed = self.cfg_speed
            self._apply_config_runtime()
        elif name == "enable":
            self._enable()
        elif name == "disable":
            self.control_word = 0x0000
            self.log("[i] 운전 정지(disable)")
        elif name == "reset":
            self._fault_reset()
        elif name == "brake":
            self._set_brake(cmd[1])
        elif name == "current":
            self._set_current(cmd[1])
        elif name == "move":
            self._move(cmd[1], relative=cmd[2], immediate=cmd[3])
        elif name == "halt":
            self.control_word = CW_ENABLE | CW_HALT
            self.log("[i] 감속정지(Halt)")

    # ---- 연결/구성 ----
    def _connect(self, ifname):
        self.log(f"[i] '{ifname}' 열기...")
        self.master = pysoem.Master()
        self.master.open(ifname)
        n = self.master.config_init()
        if n <= 0:
            self.log("[ERR] 슬레이브를 찾지 못함 → 전원/랜선/CN6A(Input) 포트 확인")
            self._teardown()
            self.status["state"] = "슬레이브 없음"
            return
        self.log(f"[i] 슬레이브 {n}개 발견")
        self.slave = self.master.slaves[0]
        self.log(f"    name={self.slave.name} man={hex(self.slave.man)} id={hex(self.slave.id)}")
        self.status["slave_name"] = self.slave.name

        # PREOP에서 PDO 매핑/모드/가감속 설정
        self.slave.config_func = self._configure
        self.master.config_map()

        if self.master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
            self.log("[ERR] SAFEOP 도달 실패")
            self._read_al_status()
            self._teardown()
            return
        self.log("[i] SAFEOP 도달")

        # OP 진입
        self.control_word = 0x0000
        self.slave.output = struct.pack("<HiI", 0, 0, self.profile_speed)
        self.master.send_processdata()
        self.master.receive_processdata(2000)
        self.master.state = pysoem.OP_STATE
        self.master.write_state()
        # OP로 올라오려면 몇 주기 PDO를 계속 보내줘야 함
        for _ in range(200):
            self.master.send_processdata()
            self.master.receive_processdata(2000)
            if self.master.state_check(pysoem.OP_STATE, 5000) == pysoem.OP_STATE:
                break
            time.sleep(0.002)
        if self.master.state_check(pysoem.OP_STATE, 5000) == pysoem.OP_STATE:
            self.op = True
            self.status["state"] = "OP (운전가능)"
            self.log("[i] ✅ OP 상태 진입 완료. Enable 후 이동 가능")
        else:
            self.log("[ERR] OP 도달 실패")
            self._read_al_status()
            self._teardown()

    def _configure(self, slave_pos):
        """PREOP에서 호출됨 → SDO로 PDO 매핑 + 모드/가감속 설정."""
        s = self.master.slaves[slave_pos]
        self.log("[i] PDO 매핑/파라미터 구성(SDO)...")

        # --- RxPDO(0x1600) 매핑: controlword, target_pos, profile_speed ---
        s.sdo_write(0x1C12, 0, struct.pack("B", 0))
        s.sdo_write(0x1600, 0, struct.pack("B", 0))
        s.sdo_write(0x1600, 1, struct.pack("<I", 0x60400010))  # 6040:00 16bit
        s.sdo_write(0x1600, 2, struct.pack("<I", 0x607A0020))  # 607A:00 32bit
        s.sdo_write(0x1600, 3, struct.pack("<I", 0x60810020))  # 6081:00 32bit
        s.sdo_write(0x1600, 0, struct.pack("B", 3))
        s.sdo_write(0x1C12, 1, struct.pack("<H", 0x1600))
        s.sdo_write(0x1C12, 0, struct.pack("B", 1))

        # --- TxPDO(0x1A00) 매핑: statusword, actual_pos, actual_speed ---
        s.sdo_write(0x1C13, 0, struct.pack("B", 0))
        s.sdo_write(0x1A00, 0, struct.pack("B", 0))
        s.sdo_write(0x1A00, 1, struct.pack("<I", 0x60410010))  # 6041:00 16bit
        s.sdo_write(0x1A00, 2, struct.pack("<I", 0x60640020))  # 6064:00 32bit
        s.sdo_write(0x1A00, 3, struct.pack("<I", 0x606C0020))  # 606C:00 32bit
        s.sdo_write(0x1A00, 0, struct.pack("B", 3))
        s.sdo_write(0x1C13, 1, struct.pack("<H", 0x1A00))
        s.sdo_write(0x1C13, 0, struct.pack("B", 1))

        # --- 운전모드 PP + 가감속 ---
        s.sdo_write(OD_MODE, 0, struct.pack("b", MODE_PP))
        s.sdo_write(OD_ACCEL, 0, struct.pack("<I", self.cfg_accel))
        s.sdo_write(OD_DECEL, 0, struct.pack("<I", self.cfg_decel))
        s.sdo_write(OD_PROFILE_SPEED, 0, struct.pack("<I", self.cfg_speed))
        self.log("[i] 구성 완료 (PP모드, "
                 f"accel={self.cfg_accel}, decel={self.cfg_decel}, speed={self.cfg_speed})")

    def _apply_config_runtime(self):
        """OP 상태에서 가감속을 SDO로 갱신(속도는 PDO로 실시간 반영)."""
        if not self.op:
            return
        try:
            self.slave.sdo_write(OD_ACCEL, 0, struct.pack("<I", self.cfg_accel))
            self.slave.sdo_write(OD_DECEL, 0, struct.pack("<I", self.cfg_decel))
            self.profile_speed = self.cfg_speed
            self.log(f"[i] 파라미터 갱신 accel={self.cfg_accel} decel={self.cfg_decel} "
                     f"speed={self.cfg_speed}")
        except Exception as e:  # noqa
            self.log(f"[ERR] 파라미터 갱신 실패: {e}")

    # ---- 상태 전이 ----
    def _io_once(self):
        """1주기 PDO 교환 후 상태워드 반환."""
        self.slave.output = struct.pack("<HiI", self.control_word & 0xFFFF,
                                        self.target_pos, self.profile_speed & 0xFFFFFFFF)
        self.master.send_processdata()
        self.master.receive_processdata(2000)
        return struct.unpack("<H", bytes(self.slave.input)[:2])[0]

    def _pump(self, cycles):
        """제어워드를 반영하며 n주기 PDO를 밀어준다."""
        for _ in range(cycles):
            self._io_once()
            time.sleep(self.CYCLE)

    def _enable(self):
        if not self.op:
            self.log("[!] OP 아님 - 먼저 연결")
            return
        self.log("[i] Enable: 폴트리셋 → 0x6→0x7→0xF 후 '운전허가됨' 확인까지 대기(~1초)")
        # 이전 폴트(폭주 등) 자동 리셋
        self.control_word = 0x0080
        self._pump(12)
        self.control_word = 0x0000
        self._pump(12)
        self.control_word = 0x0006
        self._pump(15)
        self.control_word = 0x0007
        self._pump(15)
        self.control_word = 0x000F
        # ENA(bit2) 켜질 때까지 폴링 (이 드라이버는 ~1초 걸림). 최대 3초.
        for i in range(750):
            sw = self._io_once()
            self.status["status_word"] = sw
            if sw & 0x0004:                      # bit2 Operation Enabled
                self.log(f"[i] ✅ 운전허가됨 (Operation Enabled, {int(i * self.CYCLE * 1000)}ms)")
                return
            if sw & 0x0008:                      # bit3 Error
                self.log(f"[!] Enable 중 Error 비트 (0x{sw:04X}) → Fault Reset 필요")
                return
            time.sleep(self.CYCLE)
        self.log(f"[!] 3초 내 운전허가 실패 (마지막 0x{self.status.get('status_word', 0):04X})")

    def _set_brake(self, val):
        """2403h 브레이크 출력 활성화(1)/잠금(0). 실제 해제는 Enable과 연동됨."""
        if self.slave is None:
            self.log("[!] 연결 안 됨")
            return
        try:
            self.slave.sdo_write(OD_BRAKE_ENABLE, 0, struct.pack("<H", 1 if val else 0))
            rb = struct.unpack("<H", self.slave.sdo_read(OD_BRAKE_ENABLE, 0)[:2])[0]
            self.status["brake_on"] = bool(rb)
            if val:
                self.log(f"[i] 🔓 브레이크 활성화 2403h=1 (읽기확인={rb}). "
                         f"Enable 시 {self._brake_open_delay()}ms 후 해제됨")
            else:
                self.log(f"[i] 🔒 브레이크 잠금 2403h=0 (읽기확인={rb})")
        except Exception as e:  # noqa
            self.log(f"[ERR] 브레이크 설정 실패: {e}")

    def _set_current(self, ma):
        """2303h 피크전류 상한(mA). ma=None이면 읽기만."""
        if self.slave is None:
            self.log("[!] 연결 안 됨")
            return
        try:
            if ma is not None:
                self.slave.sdo_write(0x2303, 0, struct.pack("<H", int(ma)))
            rb = struct.unpack("<H", self.slave.sdo_read(0x2303, 0)[:2])[0]
            self.status["current_ma"] = rb
            if ma is not None:
                self.log(f"[i] 전류 제한 2303h={rb}mA 설정")
        except Exception as e:  # noqa
            self.log(f"[ERR] 전류설정 실패: {e}")

    def _brake_open_delay(self):
        try:
            return struct.unpack("<H", self.slave.sdo_read(OD_BRAKE_OPEN, 0)[:2])[0]
        except Exception:
            return 200

    def _fault_reset(self):
        if not self.op:
            return
        self.log("[i] Fault Reset (0x80→0x00)")
        self.control_word = CW_FAULT_RESET
        self._pump(10)
        self.control_word = 0x0000
        self._pump(5)

    def _move(self, pos, relative, immediate):
        if not self.op:
            self.log("[!] OP 아님")
            return
        if (self.status.get("status_word", 0) & 0x0004) == 0:  # bit2 Enable Operation
            self.log("[!] 운전허가 상태 아님 - 먼저 Enable")
            return
        self.target_pos = int(pos)
        base = CW_ENABLE | (CW_RELATIVE if relative else 0)
        # bit4를 0으로 유지한 상태로 목표/모드 확정
        self.control_word = base
        self._pump(3)
        # bit4 0->1 상승엣지로 트리거
        trig = base | CW_NEW_SETPT | (CW_IMMEDIATE if immediate else 0)
        self.control_word = trig
        self._pump(3)
        # 다음 이동을 위해 bit4 내려둠 (모션은 드라이버가 내부적으로 진행)
        self.control_word = base
        kind = "상대" if relative else "절대"
        self.log(f"[i] ▶ {kind}이동 트리거 target={self.target_pos} "
                 f"{'(즉시)' if immediate else ''}")

    def _read_al_status(self):
        try:
            self.log(f"    AL status code = {hex(self.slave.al_status)}")
        except Exception:
            pass

    def _teardown(self):
        self.op = False
        if self.master is not None:
            try:
                self.master.state = pysoem.INIT_STATE
                self.master.write_state()
            except Exception:
                pass
            try:
                self.master.close()
            except Exception:
                pass
        self.master = None
        self.slave = None


# =============================================================================
#  GUI
# =============================================================================
class App:
    PPR = 10000                      # 회전당 펄스 (고정)
    ACCEL = 100000                   # 가/감속 (고정)
    COL = {"idle": "#6b7280", "wait": "#2563eb", "run": "#16a34a", "fault": "#dc2626"}

    def __init__(self, root):
        self.root = root
        root.title("TL-E EtherCAT 모터 제어")
        root.geometry("480x640")
        root.minsize(440, 600)

        self.log_q = queue.Queue()
        self.status = {"state": "미연결", "status_word": 0, "actual_pos": 0,
                       "actual_speed": 0, "error_code": 0, "slave_name": "-",
                       "wkc_ok": True, "brake_on": False, "current_ma": 0}
        self.worker = None
        self._was_fault = False
        self.v_speed_rev = tk.DoubleVar(value=1.0)     # rev/s
        self.v_step = tk.DoubleVar(value=5.0)          # deg
        self.v_current = tk.StringVar(value="3000")    # 전류 제한 mA

        st = ttk.Style()
        st.configure("Big.TButton", font=("Helvetica", 14, "bold"), padding=10)
        st.configure("TButton", padding=5)

        # ===== 큰 상태 배너 =====
        self.banner = tk.Frame(root, bg=self.COL["idle"], height=76)
        self.banner.pack(fill="x")
        self.banner.pack_propagate(False)
        self.b_state = tk.Label(self.banner, text="● 미연결", bg=self.COL["idle"],
                                fg="white", font=("Helvetica", 17, "bold"))
        self.b_state.pack(anchor="w", padx=14, pady=(10, 0))
        self.b_info = tk.Label(self.banner, text="위치 0.00 회전   ·   에러 없음",
                               bg=self.COL["idle"], fg="white", font=("Helvetica", 12))
        self.b_info.pack(anchor="w", padx=14)

        body = ttk.Frame(root)
        body.pack(fill="both", expand=True, padx=10, pady=8)

        # ===== ① 연결 =====
        f1 = ttk.LabelFrame(body, text=" ① 연결 ")
        f1.pack(fill="x", pady=4)
        row = ttk.Frame(f1); row.pack(fill="x", padx=8, pady=8)
        ttk.Label(row, text="인터페이스").pack(side="left")
        self.iface = ttk.Combobox(row, width=8, values=self._adapters())
        self.iface.set("en7")
        self.iface.pack(side="left", padx=6)
        self.btn_conn = ttk.Button(row, text="연결", command=self.on_connect)
        self.btn_conn.pack(side="left", padx=2)
        ttk.Button(row, text="해제", command=self.on_disconnect).pack(side="left", padx=2)

        # ===== ② 준비 =====
        f2 = ttk.LabelFrame(body, text=" ② 준비 ")
        f2.pack(fill="x", pady=4)
        r1 = ttk.Frame(f2); r1.pack(fill="x", padx=8, pady=(8, 2))
        ttk.Button(r1, text="🔓 브레이크 해제", command=lambda: self._post("brake", 1)).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(r1, text="🔒 브레이크 잠금", command=lambda: self._post("brake", 0)).pack(side="left", expand=True, fill="x", padx=2)
        r2 = ttk.Frame(f2); r2.pack(fill="x", padx=8, pady=(2, 6))
        ttk.Button(r2, text="⚡ 운전 ON", style="Big.TButton", command=lambda: self._post("enable")).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(r2, text="운전 OFF", command=lambda: self._post("disable")).pack(side="left", expand=True, fill="x", padx=2)
        ttk.Button(r2, text="알람리셋", command=lambda: self._post("reset")).pack(side="left", expand=True, fill="x", padx=2)
        r3 = ttk.Frame(f2); r3.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Label(r3, text="전류 제한(mA)").pack(side="left")
        ttk.Entry(r3, textvariable=self.v_current, width=6).pack(side="left", padx=4)
        for v in ("1000", "3000", "5000", "6000"):
            ttk.Button(r3, text=v, width=4, command=lambda x=v: self.v_current.set(x)).pack(side="left", padx=1)
        ttk.Button(r3, text="적용", command=self.on_current).pack(side="left", padx=4)

        # ===== ③ 이동 =====
        f3 = ttk.LabelFrame(body, text=" ③ 이동 (조그) ")
        f3.pack(fill="x", pady=4)
        sp = ttk.Frame(f3); sp.pack(fill="x", padx=8, pady=(8, 2))
        ttk.Label(sp, text="속도").pack(side="left")
        ttk.Scale(sp, from_=0.1, to=3.0, variable=self.v_speed_rev, orient="horizontal",
                  command=self._on_speed).pack(side="left", expand=True, fill="x", padx=6)
        self.lbl_speed = ttk.Label(sp, text="1.0 rev/s", width=9)
        self.lbl_speed.pack(side="left")

        stp = ttk.Frame(f3); stp.pack(fill="x", padx=8, pady=2)
        ttk.Label(stp, text="스텝").pack(side="left")
        for lbl, val in [("1°", 1), ("5°", 5), ("15°", 15), ("90°", 90), ("360°", 360)]:
            ttk.Button(stp, text=lbl, width=4, command=lambda v=val: self.v_step.set(v)).pack(side="left", padx=2)
        self.lbl_step = ttk.Label(stp, text="5°", width=6)
        self.lbl_step.pack(side="left", padx=4)

        jog = ttk.Frame(f3); jog.pack(fill="x", padx=8, pady=(4, 10))
        ttk.Button(jog, text="◀  역방향", style="Big.TButton", command=lambda: self.on_step(-1)).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(jog, text="정방향  ▶", style="Big.TButton", command=lambda: self.on_step(1)).pack(side="left", expand=True, fill="x", padx=3)

        # ===== ④ 상세/로그 =====
        f4 = ttk.LabelFrame(body, text=" 상세 · 로그 ")
        f4.pack(fill="both", expand=True, pady=4)
        self.m_detail = ttk.Label(f4, text="StatusWord ----", font=("Menlo", 10))
        self.m_detail.pack(anchor="w", padx=8, pady=(6, 2))
        self.log = tk.Text(f4, height=6, font=("Menlo", 10))
        self.log.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        if not PYSOEM_OK:
            self._log(f"[ERR] pysoem 로드 실패: {PYSOEM_ERR}  → pip3 install pysoem")

        self.root.after(150, self._poll)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---- 헬퍼 ----
    def _adapters(self):
        if not PYSOEM_OK:
            return []
        try:
            return [a.name for a in pysoem.find_adapters()]
        except Exception:
            return []

    def _ensure_worker(self):
        if self.worker is None:
            self.worker = EtherCATWorker(self.log_q, self.status)
            self.worker.start()

    def _post(self, *cmd):
        if self.worker is None:
            self._log("[!] 먼저 연결하세요")
            return
        self.worker.post(*cmd)

    def _speed_pulse(self):
        return max(200, int(self.v_speed_rev.get() * self.PPR))

    # ---- 콜백 ----
    def _on_speed(self, _=None):
        self.lbl_speed.config(text=f"{self.v_speed_rev.get():.1f} rev/s")
        if self.worker is not None:
            self._post("config", self._speed_pulse(), self.ACCEL, self.ACCEL)

    def on_connect(self):
        if not PYSOEM_OK:
            self._log("[ERR] pysoem 없음"); return
        name = self.iface.get().strip()
        if not name:
            self._log("[!] 인터페이스명을 입력하세요 (예: en7)"); return
        self._ensure_worker()
        self.worker.post("connect", name)
        self.worker.post("config", self._speed_pulse(), self.ACCEL, self.ACCEL)
        self.worker.post("current", None)      # 현재 전류제한 읽기

    def on_disconnect(self):
        self._post("disconnect")

    def on_current(self):
        try:
            ma = max(0, min(6000, int(float(self.v_current.get()))))
        except Exception:
            self._log("[!] 전류값 숫자 오류"); return
        self._post("current", ma)

    def on_step(self, direction):
        deg = self.v_step.get()
        pulses = int(round(direction * deg / 360.0 * self.PPR))
        if pulses == 0:
            self._log("[!] 스텝이 0 - 크기를 키우세요"); return
        self._log(f"[i] 조그 {'+' if direction > 0 else '-'}{abs(deg):g}° ({pulses:+d} pulse)")
        self._post("move", pulses, True, False)

    def on_close(self):
        if self.worker is not None:
            self.worker.shutdown()
            time.sleep(0.2)
        self.root.destroy()

    # ---- 표시 ----
    def _log(self, msg):
        self.log.insert("end", msg + "\n")
        self.log.see("end")

    def _poll(self):
        try:
            while True:
                self._log(self.log_q.get_nowait())
        except queue.Empty:
            pass

        s = self.status
        sw = s["status_word"]
        connected = "OP" in s["state"]
        enabled = bool(sw & 0x0004)
        fault = bool(sw & 0x0008)

        # 폴트 발생 순간 경고 (과전류 등) — 드라이버는 이미 토크 차단됨
        if fault and not self._was_fault:
            ec = s["error_code"]
            self._log(f"[⚠️폴트] 모터 정지됨! 에러 0x{ec:04X} "
                      f"{ERROR_TEXT.get(ec, '') or ''} → 원인 제거 후 '알람리셋'")
        self._was_fault = fault

        if not connected:
            key, txt = "idle", f"● {s['state']}"
        elif fault:
            key, txt = "fault", "⚠ 에러 / 폴트"
        elif enabled:
            key, txt = "run", "● 운전 중 (ON)"
        else:
            key, txt = "wait", "● 연결됨 · 운전대기"
        col = self.COL[key]
        self.banner.config(bg=col)
        self.b_state.config(text=txt, bg=col)
        ec = s["error_code"]
        errtxt = ERROR_TEXT.get(ec, "알수없음") if ec else "없음"
        rev = s["actual_pos"] / self.PPR
        self.b_info.config(bg=col,
                           text=f"위치 {rev:+.2f} 회전   ·   속도 {s['actual_speed']:+d}   ·   에러 {errtxt}")

        self.lbl_step.config(text=f"{self.v_step.get():g}°")
        brake = "🔓해제" if s.get("brake_on") else "🔒잠금"
        self.m_detail.config(
            text=f"SW 0x{sw:04X}  WKC {'OK' if s['wkc_ok'] else 'BAD'}  브레이크 {brake}  "
                 f"전류제한 {s.get('current_ma', 0)}mA  |  위치 {s['actual_pos']} pulse  에러 0x{ec:04X}")

        self.root.after(150, self._poll)


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
