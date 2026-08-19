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

import csv
import datetime
import json
import math
import os
import queue
import struct
import subprocess
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
OD_PEAK_CURRENT  = 0x2303   # U16 피크 전류 mA (모든 %의 기준, 86모델 최대 6000)
OD_HOLD_CURRENT  = 0x2307   # U16 정지(축잠금) 유지 전류 % (기본 40) = 정지 홀딩토크 결정
OD_POS_DEV_LIMIT = 0x230D   # U16 위치편차 알람 임계(펄스, 기본 4000). 초과 시 0xFF05(E5)

# ---- 리미트 스위치 / DI 관련 (매뉴얼 3.6 IO Port) --------------------------
OD_DI_FUNC        = 0x2510  # U16 I1~I5 기능설정: 0x2510+(n-1). 0:미사용 2:정방향리밋 3:역방향리밋
OD_DI_LEVEL       = 0x2500  # U16 DI 접점레벨 비트(Bit0=DI1..Bit4=DI5) 0:NO(A접점) 1:NC(B접점)
OD_DI_STATE       = 0x2100  # U16 DI 원시입력 상태 비트
OD_IO_STATUS      = 0x60FD  # I32 기능 상태(bit0:역방향리밋 bit1:정방향리밋 bit2:홈)
OD_LIMIT_STOP     = 0x2402  # U16 PP/PV 리밋 처리(0:정지 1:급정지 2:무동작)
OD_PARAM_SAVE     = 0x2201  # U16 0->1 엣지로 파라미터 저장(공장 파라미터 저장)

DI_FUNC_UNDEF     = 0
DI_FUNC_POS_LIMIT = 2       # Positive Limit
DI_FUNC_NEG_LIMIT = 3       # Negative Limit

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

# ---- 신품 입고검사 (새 모터 테스트) ----------------------------------------
# 식별 오브젝트 (CiA301 Identity Object). 서브: 1=VendorID 2=ProductCode
# 3=RevisionNo 4=SerialNumber. 4번이 개체별 고유번호인지는 모터를 물려봐야 안다.
OD_IDENTITY   = 0x1018
OD_DEVICE_NAME = 0x1008
OD_DRIVE_MODE = 0x2301   # U16 1:개루프 2:폐루프
OD_PPR        = 0x2302   # U16 회전당 펄스
OD_SUP_MODES  = 0x6502   # U32 지원 운전모드 비트
SW_TARGET_REACHED = 0x0400   # 상태워드 bit10 (CiA402 PP 목표도달)

# 기준값 — Day1(2026-07-08)에 실제로 물려서 읽은 값.
TEST_REF = {
    "man":        0xA79,     # ManufacturerID
    "product":    0x3100,    # ProductCode
    "drive_mode": 2,         # 폐루프
    "ppr":        10000,     # 회전당 펄스
    "sup_modes":  0xA5,      # PP·PV·HM·CSP
}

# 합격/불합격 임계값.
# ⚠️ 아래 숫자는 아직 실측 근거가 없는 초기값이다(추측). 정상 모터 2~3대를
#    먼저 돌려서 실제 측정치를 보고 조정할 것. 같은 폴더의 test_limits.json 을
#    만들어 두면 코드를 고치지 않고 덮어쓸 수 있다.
TEST_LIMITS = {
    "enable_ms_min":    100,    # 운전허가까지 걸린 시간 하한 (Day1 실측 ≈1009ms)
    "enable_ms_max":    2500,   # 상한
    "idle_jitter_max":  60,     # 정지 중 위치 흔들림 pulse
    "err_1rev":         80,     # 1회전 왕복 후 복귀 오차 pulse
    "err_5rev":         150,    # 5회전 왕복 후
    "err_sweep":        300,    # 속도 스윕 후
    "err_stress":       400,    # 급가감속 반복 후
    "err_final":        500,    # 전체 종료 후 원점 복귀 오차
    "speed_tol_pct":    12,     # 지령 속도 대비 실측 최고속도 오차 %
    "ferr_max":         2500,   # 이동 중 최대 추종오차 pulse
}

# 속도 스윕 항목 (rev/s). 이동량은 그 속도에 실제로 도달하도록 자동 계산한다.
TEST_SWEEP_SPEEDS = (3.0, 8.0, 15.0)
TEST_ACCEL_NORMAL = 100000    # 일반 구간 가속 pulse/s^2
TEST_ACCEL_SWEEP  = 300000    # 스윕: 짧은 거리에서 목표속도 도달용
TEST_ACCEL_STRESS = 600000    # 급가감속 스트레스
TEST_RESULT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_results")
TEST_CSV = os.path.join(TEST_RESULT_DIR, "motor_test.csv")


def load_test_limits():
    """test_limits.json 이 있으면 임계값을 덮어쓴다(코드 수정 없이 튜닝)."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_limits.json")
    lim = dict(TEST_LIMITS)
    try:
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                lim.update(json.load(f))
    except Exception:
        pass
    return lim


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
        self._estop_ev = threading.Event()   # 비상정지: GUI 스레드에서 즉시 set

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
        self._di_warned = False

        # 리미트: 각 방향에 현재 배정된 DI 채널(0..4), 재배정 시 이전 DI 해제용
        self.limit_di = {"pos": None, "neg": None}

        # 끝단감지(위치편차): 추종오차 60F4h가 임계 넘으면 Halt(홀딩유지). 폴트 전에 잡음.
        self.hardstop_on = False
        self.hardstop_thr = 3000
        self.hardstop_latched = False

    # ---- 외부(GUI)에서 호출하는 명령 인터페이스 ----
    def post(self, *cmd):
        self.cmd_q.put(cmd)

    def shutdown(self):
        self._stopev.set()

    def trigger_estop(self):
        """GUI 스레드에서 직접 호출 — 큐를 거치지 않고 즉시 토크 차단.
        (int 대입/Event.set은 스레드 안전) 조그 _pump 루프도 이 이벤트로 즉시 중단."""
        self.control_word = 0x0000
        self._estop_ev.set()

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

    def _parse_tx(self):
        """수신한 TxPDO를 status에 반영하고 추종오차를 반환.
        TxPDO: [statusword U16][actual_pos I32][actual_speed I32]
               [error_code U16][io_status I32][follow_err I32]  = 20 bytes
        (_cycle 과 셀프테스트 양쪽에서 쓰므로 함수로 분리)"""
        data = bytes(self.slave.input)
        self.status["tx_len"] = len(data)   # 60F4(추종오차) 매핑 여부 판별용
        ferr = self.status.get("follow_err", 0)
        if len(data) >= 20:
            sw, apos, aspd, ec, io, ferr = struct.unpack("<HiiHii", data[:20])
            self.status["follow_err"] = ferr
        elif len(data) >= 16:  # 60F4 미매핑 폴백
            sw, apos, aspd, ec, io = struct.unpack("<HiiHi", data[:16])
        elif len(data) >= 10:  # 매핑 실패 등 폴백
            sw, apos, aspd = struct.unpack("<Hii", data[:10])
            self.status["status_word"] = sw
            self.status["actual_pos"] = apos
            self.status["actual_speed"] = aspd
            return ferr
        else:
            return ferr
        self.status["status_word"] = sw
        self.status["actual_pos"] = apos
        self.status["actual_speed"] = aspd
        self.status["error_code"] = ec
        self.status["io_status"] = io
        self.status["limit_neg"] = bool(io & 0x1)   # bit0 역방향 리밋
        self.status["limit_pos"] = bool(io & 0x2)   # bit1 정방향 리밋
        self.status["limit_home"] = bool(io & 0x4)  # bit2 홈
        return ferr

    # ---- 한 주기 PDO 교환 ----
    def _cycle(self):
        # 비상정지 처리: 토크 차단 유지, 잔여 목표 제거, 1회 로그
        if self._estop_ev.is_set():
            self._estop_ev.clear()
            self.control_word = 0x0000
            self.target_pos = self.status.get("actual_pos", self.target_pos)
            self.log("[🛑] 비상정지! 토크 차단(controlword=0). 재가동하려면 '운전 ON'")
        # RxPDO: [controlword U16][target_pos I32][profile_speed U32]
        self.slave.output = struct.pack("<HiI", self.control_word & 0xFFFF,
                                        self.target_pos, self.profile_speed & 0xFFFFFFFF)
        self.master.send_processdata()
        wkc = self.master.receive_processdata(2000)
        if wkc < 1:
            self.status["wkc_ok"] = False
        else:
            self.status["wkc_ok"] = True

        ferr = self._parse_tx()

        # 끝단감지: 추종오차가 소프트 임계 초과 → 폴트(토크차단=하중낙하) 전에 Halt(홀딩유지)
        if (self.hardstop_on and not self.hardstop_latched
                and (self.status.get("status_word", 0) & 0x0004)   # 운전허가 중일 때만
                and abs(ferr) > self.hardstop_thr):
            self.control_word = CW_ENABLE | CW_HALT
            self.hardstop_latched = True
            self.status["hardstop_hit"] = True
            self.log(f"[🛑끝단] 위치편차 {ferr} > {self.hardstop_thr} → 정지(홀딩유지, "
                     "하중 안떨어짐). 반대로 이동하세요")

        # 원시 DI 입력상태(2100h)를 약 0.4초마다 SDO로 읽음.
        # 60FD(기능상태, 기능배정+저장 필요)와 달리 단자에 신호가 물리적으로
        # 들어오는지 그대로 보여줌 → 센서/배선 진단의 결정적 근거.
        self._err_poll += 1
        if self._err_poll >= 100:
            self._err_poll = 0
            try:
                di = struct.unpack("<H", self.slave.sdo_read(OD_DI_STATE, 0)[:2])[0]
                self.status["di_raw"] = di
            except Exception as e:  # noqa
                if not self._di_warned:
                    self._di_warned = True
                    self.log(f"[!] 2100h(DI상태) 읽기 실패: {e}")
            # 에러코드(603F) SDO 교차확인 — PDO값이 0인데 여기서 뜨면 PDO매핑 문제 판별
            try:
                self.status["error_sdo"] = struct.unpack(
                    "<H", self.slave.sdo_read(OD_ERROR_CODE, 0)[:2])[0]
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
        elif name == "holdcur":
            self._set_holdcur(cmd[1])
        elif name == "posdev":
            self._set_posdev(cmd[1])
        elif name == "hardstop":
            self._cfg_hardstop(cmd[1], cmd[2])
        elif name == "move":
            self._move(cmd[1], relative=cmd[2], immediate=cmd[3])
        elif name == "halt":
            self.control_word = CW_ENABLE | CW_HALT
            self.log("[i] 감속정지(Halt)")
        elif name == "jog_start":
            self._jog_start(cmd[1], cmd[2])
        elif name == "jog_stop":
            self._jog_stop()
        elif name == "estop":
            self.trigger_estop()
        elif name == "set_limit":
            self._set_limit(cmd[1], cmd[2], cmd[3], cmd[4])
        elif name == "limit_mode":
            self._set_limit_mode(cmd[1])
        elif name == "limit_read":
            self._read_limit_cfg()
        elif name == "save_params":
            self._save_params()
        elif name == "selftest":
            self._run_selftest(cmd[1], cmd[2])

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

        # --- TxPDO(0x1A00) 매핑: statusword, actual_pos, actual_speed,
        #     error_code(603F), IO_status(60FD) ---
        #     603F/60FD는 매뉴얼 PP모드 권장 매핑. SDO 폴링(예외에 0으로 남음) 대신
        #     매 주기 확실히 읽기 위해 PDO로 매핑한다.
        s.sdo_write(0x1C13, 0, struct.pack("B", 0))
        s.sdo_write(0x1A00, 0, struct.pack("B", 0))
        s.sdo_write(0x1A00, 1, struct.pack("<I", 0x60410010))  # 6041:00 16bit statusword
        s.sdo_write(0x1A00, 2, struct.pack("<I", 0x60640020))  # 6064:00 32bit actual_pos
        s.sdo_write(0x1A00, 3, struct.pack("<I", 0x606C0020))  # 606C:00 32bit actual_speed
        s.sdo_write(0x1A00, 4, struct.pack("<I", 0x603F0010))  # 603F:00 16bit 에러코드
        s.sdo_write(0x1A00, 5, struct.pack("<I", 0x60FD0020))  # 60FD:00 32bit IO상태(리밋/홈)
        s.sdo_write(0x1A00, 6, struct.pack("<I", 0x60F40020))  # 60F4:00 32bit 추종오차(끝단감지)
        s.sdo_write(0x1A00, 0, struct.pack("B", 6))
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
        """1주기 PDO 교환 후 상태워드 반환.
        (_parse_tx 로 status 전체를 갱신하므로 셀프테스트처럼 _cycle 이 멈춘
         구간에서도 화면의 위치·추종오차가 계속 살아 있다)"""
        self.slave.output = struct.pack("<HiI", self.control_word & 0xFFFF,
                                        self.target_pos, self.profile_speed & 0xFFFFFFFF)
        self.master.send_processdata()
        self.master.receive_processdata(2000)
        self._parse_tx()
        return self.status.get("status_word", 0)

    def _pump(self, cycles):
        """제어워드를 반영하며 n주기 PDO를 밀어준다. 비상정지 시 즉시 중단."""
        for _ in range(cycles):
            if self._estop_ev.is_set():
                self.control_word = 0x0000
                break
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
        # 경과시간은 벽시계로 잰다(주기 곱셈은 PDO 왕복시간이 빠져 실제보다 짧게 나옴).
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            sw = self._io_once()
            if sw & 0x0004:                      # bit2 Operation Enabled
                ms = int((time.monotonic() - t0) * 1000)
                self.log(f"[i] ✅ 운전허가됨 (Operation Enabled, {ms}ms)")
                return ms
            if sw & 0x0008:                      # bit3 Error
                self.log(f"[!] Enable 중 Error 비트 (0x{sw:04X}) → Fault Reset 필요")
                return None
            time.sleep(self.CYCLE)
        self.log(f"[!] 3초 내 운전허가 실패 (마지막 0x{self.status.get('status_word', 0):04X})")
        return None

    # ---- 끝단감지(위치편차) ----
    def _cfg_hardstop(self, on, thr):
        self.hardstop_on = bool(on)
        self.hardstop_thr = max(200, int(thr))
        self.hardstop_latched = False
        self.status["hardstop_hit"] = False
        self.log(f"[i] 끝단감지 {'ON' if on else 'OFF'} (임계 {self.hardstop_thr}펄스). "
                 "감지 시 정지·홀딩유지(폴트 아님)")

    # ---- 리미트 스위치 ----
    def _set_limit(self, side, ch, enabled, nc):
        """한 방향(side='pos'/'neg') 리미트를 DI 단자(ch=0..4, DI0~DI4)에 배정/해제.
        - 2510h+ch 에 기능값(2:정방향 3:역방향, 해제 시 0) 기록
        - 2500h 의 비트 ch 로 NO/NC 접점 설정
        - DI를 바꾼 경우 이전 DI 기능을 0으로 되돌림
        """
        if self.slave is None:
            self.log("[!] 연결 안 됨"); return
        func = (DI_FUNC_POS_LIMIT if side == "pos" else DI_FUNC_NEG_LIMIT) if enabled else DI_FUNC_UNDEF
        label = "정방향(+)" if side == "pos" else "역방향(−)"
        try:
            prev = self.limit_di.get(side)
            if prev is not None and prev != ch:
                self.slave.sdo_write(OD_DI_FUNC + prev, 0, struct.pack("<H", DI_FUNC_UNDEF))
            self.slave.sdo_write(OD_DI_FUNC + ch, 0, struct.pack("<H", func))
            # NO/NC (2500h 비트 ch read-modify-write)
            lvl = struct.unpack("<H", self.slave.sdo_read(OD_DI_LEVEL, 0)[:2])[0]
            if nc:
                lvl |= (1 << ch)
            else:
                lvl &= ~(1 << ch)
            self.slave.sdo_write(OD_DI_LEVEL, 0, struct.pack("<H", lvl & 0xFFFF))
            self.limit_di[side] = ch if enabled else None
            rb = struct.unpack("<H", self.slave.sdo_read(OD_DI_FUNC + ch, 0)[:2])[0]
            if enabled:
                self.log(f"[i] {label} 리밋 → DI{ch} {'NC(B접점)' if nc else 'NO(A접점)'} "
                         f"기능 {hex(OD_DI_FUNC + ch)}={rb}(기대 {func}). "
                         f"※ 미반영 시 '💾 저장' 후 전원 재기동")
            else:
                self.log(f"[i] {label} 리밋 해제 (DI{ch} 기능=0)")
        except Exception as e:  # noqa
            self.log(f"[ERR] 리밋 설정 실패: {e}")

    def _set_limit_mode(self, mode):
        """2402h PP/PV 리밋 처리(0:정지 1:급정지 2:무동작)."""
        if self.slave is None:
            self.log("[!] 연결 안 됨"); return
        try:
            self.slave.sdo_write(OD_LIMIT_STOP, 0, struct.pack("<H", int(mode)))
            rb = struct.unpack("<H", self.slave.sdo_read(OD_LIMIT_STOP, 0)[:2])[0]
            names = {0: "정지", 1: "급정지", 2: "무동작"}
            self.log(f"[i] 리밋 동작 2402h={rb} ({names.get(rb, '?')})")
        except Exception as e:  # noqa
            self.log(f"[ERR] 리밋 동작 설정 실패: {e}")

    def _read_limit_cfg(self):
        """현재 DI 기능/접점/동작 설정을 읽어 GUI 동기화용으로 status에 저장."""
        if self.slave is None:
            return
        try:
            funcs = [struct.unpack("<H", self.slave.sdo_read(OD_DI_FUNC + n, 0)[:2])[0]
                     for n in range(5)]
            lvl = struct.unpack("<H", self.slave.sdo_read(OD_DI_LEVEL, 0)[:2])[0]
            mode = struct.unpack("<H", self.slave.sdo_read(OD_LIMIT_STOP, 0)[:2])[0]
            self.limit_di["pos"] = next((i for i, f in enumerate(funcs)
                                         if f == DI_FUNC_POS_LIMIT), None)
            self.limit_di["neg"] = next((i for i, f in enumerate(funcs)
                                         if f == DI_FUNC_NEG_LIMIT), None)
            self.status["limit_cfg"] = {"funcs": funcs, "level": lvl, "mode": mode}
            self.status["limit_cfg_dirty"] = True
            self.log(f"[i] DI 기능현황 2510~2514h={funcs}(2:+리밋 3:-리밋) "
                     f"2500h=0x{lvl:02X} 2402h(동작)={mode}")
        except Exception as e:  # noqa
            self.log(f"[ERR] 리밋 설정 읽기 실패: {e}")

    def _save_params(self):
        """2201h 0->1 엣지로 파라미터를 드라이버에 저장."""
        if self.slave is None:
            self.log("[!] 연결 안 됨"); return
        try:
            self.slave.sdo_write(OD_PARAM_SAVE, 0, struct.pack("<H", 0))
            time.sleep(0.05)
            self.slave.sdo_write(OD_PARAM_SAVE, 0, struct.pack("<H", 1))
            self.log("[i] 💾 파라미터 저장(2201h 0→1). 리밋 기능은 저장 후 "
                     "전원 재기동 시 확실히 반영됩니다")
        except Exception as e:  # noqa
            self.log(f"[ERR] 파라미터 저장 실패: {e}")

    def _set_posdev(self, val):
        """230Dh 위치편차 알람 임계(펄스). val=None이면 읽기만.
        너무 낮으면 리밋 정지/감속 지연에도 0xFF05 나기 쉬움. 너무 높이면 충돌 감지 둔해짐."""
        if self.slave is None:
            self.log("[!] 연결 안 됨")
            return
        try:
            if val is not None:
                self.slave.sdo_write(OD_POS_DEV_LIMIT, 0, struct.pack("<H", int(val)))
            rb = struct.unpack("<H", self.slave.sdo_read(OD_POS_DEV_LIMIT, 0)[:2])[0]
            self.status["pos_dev"] = rb
            if val is None:
                self.status["pos_dev_sync"] = True   # 읽기 → GUI 엔트리 1회 반영
            if val is not None:
                self.log(f"[i] 위치편차 임계 230Dh={rb}펄스 ({rb / 10000:.2f}회전). "
                         f"클수록 리밋정지 시 0xFF05 덜 남(대신 충돌감지 둔해짐)")
        except Exception as e:  # noqa
            self.log(f"[ERR] 위치편차 임계 설정 실패: {e}")

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

    def _set_holdcur(self, pct):
        """2307h 정지(축잠금) 홀딩 전류 %. pct=None이면 읽기만.
        정지 시 유지력(홀딩토크)을 결정. 실제 전류 ≈ pct% × 2303h(peak).
        주의: 전자 홀딩은 전원 나가면 0 — 무전원 유지가 필요하면 브레이크 대체 불가."""
        if self.slave is None:
            self.log("[!] 연결 안 됨")
            return
        try:
            if pct is not None:
                self.slave.sdo_write(OD_HOLD_CURRENT, 0, struct.pack("<H", int(pct)))
            rb = struct.unpack("<H", self.slave.sdo_read(OD_HOLD_CURRENT, 0)[:2])[0]
            peak = struct.unpack("<H", self.slave.sdo_read(OD_PEAK_CURRENT, 0)[:2])[0]
            self.status["hold_pct"] = rb
            self.status["hold_ma"] = int(peak * rb / 100)
            if pct is not None:
                self.log(f"[i] 정지 홀딩전류 2307h={rb}% "
                         f"(≈{self.status['hold_ma']}mA / peak {peak}mA). "
                         f"손으로 축 돌려 유지력 확인하세요")
        except Exception as e:  # noqa
            self.log(f"[ERR] 홀딩전류 설정 실패: {e}")

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
        # 새 이동 → 끝단감지 래치 해제(반대로 빠져나올 수 있게)
        self.hardstop_latched = False
        self.status["hardstop_hit"] = False
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

    def _jog_start(self, direction, big):
        """버튼을 누르는 동안 계속 이동: 아주 큰 상대이동을 걸고, 뗄 때 Halt로 정지.
        (무한이 아닌 1000회전 상한 → 이벤트 유실 시에도 폭주 방지)"""
        if not self.op:
            self.log("[!] OP 아님"); return
        if (self.status.get("status_word", 0) & 0x0004) == 0:
            self.log("[!] 운전허가 아님 - 먼저 Enable"); return
        self._move(direction * int(big), relative=True, immediate=True)
        self.log(f"[i] ▶▶ 연속조그 {'정방향(들어옴)' if direction > 0 else '역방향(나감)'} "
                 f"시작 (버튼 유지 중 이동)")

    def _jog_stop(self):
        """버튼을 떼면 Halt(bit8)로 감속정지. control_word는 매 주기 유지 전송됨."""
        if not self.op:
            return
        self.control_word = CW_ENABLE | CW_HALT
        self.log("[i] ■ 연속조그 정지(Halt)")

    # =====================================================================
    #  신품 입고검사 — "새 모터 테스트"
    # =====================================================================
    def _sdo_num(self, index, sub, fmt, size):
        """SDO 한 항목을 숫자로 읽는다. 실패하면 None (미구현 오브젝트 판별용)."""
        try:
            return struct.unpack(fmt, self.slave.sdo_read(index, sub)[:size])[0]
        except Exception:
            return None

    def _read_identity(self):
        """식별정보. 1018h:04 시리얼이 개체마다 다르면 그게 진짜 하드웨어 아이디다.
        (이 드라이버가 시리얼을 넣어놨는지는 모터를 물려봐야 안다 → 값 그대로 기록)
        ※ ESC EEPROM(Station Alias)은 건드리지 않는다. EEPROM 워드 7의 체크섬이
          워드 0~6을 덮으므로, 별도 계산 없이 alias만 쓰면 슬레이브가 인식 안 될
          수 있다. 신품 42대에 걸 위험이 아니다."""
        idn = {}
        try:
            idn["name"] = self.slave.name
            idn["man"] = self.slave.man
            idn["product"] = self.slave.id
            idn["revision"] = self.slave.rev
        except Exception:
            idn.setdefault("name", "?")
        idn["serial"] = self._sdo_num(OD_IDENTITY, 4, "<I", 4)
        return idn

    def _t_item(self, res, name, ok, detail, critical=True):
        """검사 항목 하나를 기록. critical=False 면 불합격이어도 경고로만 본다."""
        res["items"].append({"name": name, "ok": bool(ok), "detail": detail,
                             "critical": critical})
        mark = "✅" if ok else ("❌" if critical else "⚠️")
        self.log(f"    {mark} {name} — {detail}")
        self.status["test_items"] = list(res["items"])

    def _wait_stopped(self, limit=2.0):
        """축이 실제로 멈출 때까지 기다린다.
        이게 없으면 앞 이동이 아직 감속 중인데 다음 이동을 걸어버려서, 다음
        이동의 '출발 위치'와 '최고속도'가 전부 엉킨다(2026-08-19 실측으로 확인)."""
        t0 = time.monotonic()
        low = 0
        while time.monotonic() - t0 < limit:
            self._io_once()
            if abs(self.status.get("actual_speed", 0)) < 100:
                low += 1
                if low >= 15:      # 60ms 연속 정지
                    return True
            else:
                low = 0
            time.sleep(self.CYCLE)
        return False

    def _move_timeout(self, delta, v, accel):
        """사다리꼴 프로파일 예상 소요시간의 3배 + 1초를 타임아웃으로 쓴다."""
        d = abs(delta)
        d_ramp = v * v / accel                    # 가속거리 + 감속거리
        if d > d_ramp:
            t = 2.0 * v / accel + (d - d_ramp) / v
        else:
            t = 2.0 * math.sqrt(d / accel)
        return max(2.0, t * 3.0 + 1.0)

    def _t_ferr(self, res, name, val, limit):
        """추종오차 판정. 60F4h가 TxPDO에 안 실렸으면 '측정 못 함'으로 남긴다.
        (값 0을 합격으로 적으면 측정 안 된 걸 통과로 착각하게 된다)"""
        if self.status.get("tx_len", 0) < 20:
            self._t_item(res, name, True,
                         "60F4h 미매핑 → 측정 못 함(합격 근거 아님)", critical=False)
        else:
            self._t_item(res, name, val <= limit, f"{val} pulse (허용 {limit})")

    def _test_move(self, delta, speed_rev, accel):
        """상대이동 1회 + 완전히 멈출 때까지 대기하며 실측.
        완료 판정은 '속도가 0이고 목표 근처'를 160ms 연속 만족할 때만 인정한다.
        속도가 한 번 낮아졌다고 바로 끝내면 감속 중에 위치를 읽어버린다.
        반환: dict(pos_err, max_ferr, max_speed, fault, aborted, reached_bit, secs)"""
        # 앞 이동이 완전히 멈춘 뒤에 출발 위치를 읽는다(엉킴 방지의 핵심)
        self._wait_stopped()
        start = self.status.get("actual_pos", 0)
        target = start + delta
        v = max(200.0, abs(speed_rev) * App.PPR)
        self.profile_speed = int(v)
        try:
            self.slave.sdo_write(OD_ACCEL, 0, struct.pack("<I", int(accel)))
            self.slave.sdo_write(OD_DECEL, 0, struct.pack("<I", int(accel)))
        except Exception as e:  # noqa
            self.log(f"[!] 가감속 설정 실패: {e}")
        timeout = self._move_timeout(delta, v, accel)
        tol = max(50, int(abs(delta) * 0.002))    # 도달로 인정할 위치 여유

        self._move(delta, relative=True, immediate=True)
        t0 = time.monotonic()
        max_ferr = 0
        max_spd = 0
        started = False
        reached_bit = False
        low = 0                  # 저속이 몇 주기 연속인지
        why = "타임아웃"
        while time.monotonic() - t0 < timeout:
            if self._estop_ev.is_set():
                return {"aborted": True, "pos_err": 0, "max_ferr": max_ferr,
                        "max_speed": max_spd, "fault": 0, "reached_bit": reached_bit,
                        "secs": 0, "why": "비상정지"}
            self._io_once()
            sw = self.status.get("status_word", 0)
            pos = self.status.get("actual_pos", 0)
            spd = abs(self.status.get("actual_speed", 0))
            max_spd = max(max_spd, spd)
            max_ferr = max(max_ferr, abs(self.status.get("follow_err", 0)))
            if sw & SW_TARGET_REACHED:
                reached_bit = True
            if self.status.get("error_code", 0):
                why = "폴트"
                break
            if spd > 500:
                started = True
            low = low + 1 if spd < 100 else 0
            near = abs(target - pos) <= tol
            # 드라이버가 목표도달(bit10)을 주면 그게 가장 확실한 근거다.
            # 안 주는 펌웨어도 있으므로 '정지 100ms 연속 + 목표 근처'를 대안으로 둔다.
            if started and near and (low >= 25 or (reached_bit and low >= 8)):
                why = "정상도달"
                break
            if started and low >= 100:
                why = "멈췄으나 목표 못감"   # 0.4초 정지인데 목표와 멀다
                break
            if not started and low >= 250:
                why = "안 움직임"
                break
            time.sleep(self.CYCLE)
        secs = time.monotonic() - t0
        self._pump(25)          # 100ms 추가 정착
        end = self.status.get("actual_pos", 0)
        r = {"aborted": False, "pos_err": end - target, "max_ferr": max_ferr,
             "max_speed": max_spd, "fault": self.status.get("error_code", 0),
             "reached_bit": reached_bit, "secs": secs, "why": why}
        # 이동 하나하나의 실측을 로그에 남긴다(판정이 이상할 때 근거가 됨)
        self.log(f"      · {delta:+d}p @{speed_rev:g}rev/s → 최고 "
                 f"{max_spd / App.PPR:.2f}rev/s, {secs:.2f}초, 도달오차 "
                 f"{r['pos_err']:+d}p, bit10={'Y' if reached_bit else 'N'}, {why}")
        return r

    def _run_selftest(self, motor_id, note):
        if not self.op:
            self.log("[!] OP 아님 - 먼저 ① 연결")
            return
        if self.status.get("test_running"):
            self.log("[!] 이미 검사 중")
            return
        lim = load_test_limits()
        res = {"id": motor_id, "note": note, "items": [], "meas": {}}
        self.status["test_running"] = True
        self.status["test_items"] = []
        self.status["test_verdict"] = ""
        self.status["test_id"] = motor_id
        self._estop_ev.clear()
        t_start = time.monotonic()
        self.log(f"\n===== 새 모터 테스트 시작  [{motor_id}] =====")
        try:
            self._selftest_body(res, lim)
        except Exception as e:  # noqa
            self._t_item(res, "검사 중 예외", False, str(e))
        finally:
            # 어떤 경우에도 토크 차단하고 끝낸다(모터 교체 안전)
            self.control_word = 0x0000
            try:
                self._pump(10)
            except Exception:
                pass
            res["meas"]["elapsed_s"] = round(time.monotonic() - t_start, 1)
            self._finish_selftest(res)
            self.status["test_running"] = False
            self.status["test_step"] = ""

    def _selftest_body(self, res, lim):
        PPR = App.PPR
        M = res["meas"]
        N = 9

        def step(i, title):
            if self._estop_ev.is_set():
                raise RuntimeError("비상정지로 중단됨")
            self.status["test_step"] = f"{i}/{N} {title}"
            self.log(f"[{i}/{N}] {title}")

        # ---- 1. 통신·식별 -------------------------------------------------
        step(1, "통신·식별")
        idn = self._read_identity()
        M.update(idn)
        man, prod, rev = idn.get("man"), idn.get("product"), idn.get("revision")
        self._t_item(res, "제조사ID", man == TEST_REF["man"],
                     f"{'0x%X' % man if man is not None else '읽기실패'} "
                     f"(기준 0x{TEST_REF['man']:X})")
        self._t_item(res, "제품코드", prod == TEST_REF["product"],
                     f"{'0x%X' % prod if prod is not None else '읽기실패'} "
                     f"(기준 0x{TEST_REF['product']:X})", critical=False)
        self._t_item(res, "리비전", rev is not None,
                     f"{'0x%X' % rev if rev is not None else '읽기실패'}", critical=False)
        self._io_once()                      # TxPDO 길이 확인용 1주기
        txl = self.status.get("tx_len", 0)
        M["tx_len"] = txl
        self._t_item(res, "TxPDO 길이", txl >= 20,
                     f"{txl}바이트 — "
                     + ("60F4h(추종오차)까지 실림" if txl >= 20
                        else "60F4h 안 실림 → 추종오차·끝단감지 측정 불가"),
                     critical=False)
        ser = idn.get("serial")
        self._t_item(res, "하드웨어 시리얼(1018h:04)", True,
                     f"{ser}" if ser else "없음/미구현 → 아이디는 사람이 붙인 번호로만 관리",
                     critical=False)

        # ---- 2. 공장 파라미터 ---------------------------------------------
        step(2, "공장 파라미터")
        dm = self._sdo_num(OD_DRIVE_MODE, 0, "<H", 2)
        ppr = self._sdo_num(OD_PPR, 0, "<H", 2)
        peak = self._sdo_num(OD_PEAK_CURRENT, 0, "<H", 2)
        sup = self._sdo_num(OD_SUP_MODES, 0, "<I", 4)
        M.update({"drive_mode": dm, "ppr": ppr, "peak_ma": peak, "sup_modes": sup})
        self._t_item(res, "운전모드 2301h", dm == TEST_REF["drive_mode"],
                     f"{dm} (기준 {TEST_REF['drive_mode']}=폐루프)")
        self._t_item(res, "회전당 펄스 2302h", ppr == TEST_REF["ppr"],
                     f"{ppr} (기준 {TEST_REF['ppr']})")
        self._t_item(res, "피크전류 2303h", peak is not None, f"{peak} mA", critical=False)
        self._t_item(res, "지원모드 6502h", sup == TEST_REF["sup_modes"],
                     f"{'0x%X' % sup if sup is not None else '읽기실패'} "
                     f"(기준 0x{TEST_REF['sup_modes']:X})", critical=False)

        # ---- 3. 운전허가 시간 ---------------------------------------------
        # 폐루프 정렬에 걸리는 시간. 너무 길거나 짧으면 엔코더/정렬 이상 신호.
        step(3, "운전허가(Enable) 시간")
        ms = self._enable()
        M["enable_ms"] = ms
        self._t_item(res, "운전허가 시간",
                     ms is not None and lim["enable_ms_min"] <= ms <= lim["enable_ms_max"],
                     f"{ms}ms (허용 {lim['enable_ms_min']}~{lim['enable_ms_max']})"
                     if ms is not None else "운전허가 실패")
        if ms is None:
            raise RuntimeError("운전허가가 안 되어 이후 검사를 못 함")

        # ---- 4. 정지 안정성 -----------------------------------------------
        # 명령 없이 서 있을 때 위치가 흔들리면 엔코더나 정류에 문제가 있다.
        step(4, "정지 안정성")
        lo = hi = self.status.get("actual_pos", 0)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.6:
            self._io_once()
            p = self.status.get("actual_pos", 0)
            lo = min(lo, p); hi = max(hi, p)
            time.sleep(self.CYCLE)
        jit = hi - lo
        M["idle_jitter"] = jit
        self._t_item(res, "정지 중 흔들림", jit <= lim["idle_jitter_max"],
                     f"{jit} pulse (허용 {lim['idle_jitter_max']})")

        origin = self.status.get("actual_pos", 0)
        M["origin"] = origin

        # ---- 5. 저속 1회전 왕복 -------------------------------------------
        step(5, "저속 1회전 정·역")
        a = self._test_move(+PPR, 1.0, TEST_ACCEL_NORMAL)
        b = self._test_move(-PPR, 1.0, TEST_ACCEL_NORMAL)
        if a["aborted"] or b["aborted"]:
            raise RuntimeError("비상정지로 중단됨")
        err1 = abs(self.status.get("actual_pos", 0) - origin)
        f1 = max(a["max_ferr"], b["max_ferr"])
        M.update({"err_1rev": err1, "ferr_1rev": f1,
                  "bit10_seen": bool(a["reached_bit"] or b["reached_bit"])})
        self._t_item(res, "1회전 왕복 복귀오차", err1 <= lim["err_1rev"],
                     f"{err1} pulse (허용 {lim['err_1rev']})")
        self._t_ferr(res, "1회전 최대 추종오차", f1, lim["ferr_max"])
        self._chk_fault(res, "1회전 구간 폴트")

        # ---- 6. 다회전 ----------------------------------------------------
        step(6, "5회전 정·역")
        a = self._test_move(+5 * PPR, 5.0, TEST_ACCEL_NORMAL)
        b = self._test_move(-5 * PPR, 5.0, TEST_ACCEL_NORMAL)
        if a["aborted"] or b["aborted"]:
            raise RuntimeError("비상정지로 중단됨")
        err5 = abs(self.status.get("actual_pos", 0) - origin)
        f5 = max(a["max_ferr"], b["max_ferr"])
        M.update({"err_5rev": err5, "ferr_5rev": f5})
        self._t_item(res, "5회전 왕복 복귀오차", err5 <= lim["err_5rev"],
                     f"{err5} pulse (허용 {lim['err_5rev']})")
        self._t_ferr(res, "5회전 최대 추종오차", f5, lim["ferr_max"])
        self._chk_fault(res, "5회전 구간 폴트")

        # ---- 7. 속도 스윕 --------------------------------------------------
        # 이동량을 속도에 비례시켜, 가감속 구간만으로 끝나지 않고 지령속도에
        # 실제로 도달하게 한다(짧은 이동이면 최고속도에 못 닿아 측정이 무의미).
        step(7, "속도 스윕")
        sweep = []
        for v in TEST_SWEEP_SPEEDS:
            revs = max(2.0, v * 0.9)
            fw = self._test_move(int(+revs * PPR), v, TEST_ACCEL_SWEEP)
            bw = self._test_move(int(-revs * PPR), v, TEST_ACCEL_SWEEP)
            if fw["aborted"] or bw["aborted"]:
                raise RuntimeError("비상정지로 중단됨")
            want = v * PPR
            got = max(fw["max_speed"], bw["max_speed"])
            dev = abs(got - want) / want * 100.0
            ferr = max(fw["max_ferr"], bw["max_ferr"])
            sweep.append((v, got, dev, ferr))
            self._t_item(res, f"{v:g} rev/s 속도", dev <= lim["speed_tol_pct"],
                         f"실측 {got / PPR:.2f} rev/s (오차 {dev:.1f}%, "
                         f"허용 {lim['speed_tol_pct']}%)")
            self._t_ferr(res, f"{v:g} rev/s 추종오차", ferr, lim["ferr_max"])
        M["sweep"] = "; ".join(f"{v:g}rev/s→{g / PPR:.2f}({d:.0f}%,fe{f})"
                               for v, g, d, f in sweep)
        errs = abs(self.status.get("actual_pos", 0) - origin)
        M["err_sweep"] = errs
        self._t_item(res, "스윕 후 복귀오차", errs <= lim["err_sweep"],
                     f"{errs} pulse (허용 {lim['err_sweep']})")
        self._chk_fault(res, "스윕 구간 폴트")

        # ---- 8. 급가감속 반복 ----------------------------------------------
        # Day1에 폭주가 났던 조건(고가속 출발)을 일부러 재현해 탈조를 잡는다.
        step(8, "급가감속 반복(4회 왕복)")
        fmax = 0
        for _ in range(4):
            a = self._test_move(+2 * PPR, 10.0, TEST_ACCEL_STRESS)
            b = self._test_move(-2 * PPR, 10.0, TEST_ACCEL_STRESS)
            if a["aborted"] or b["aborted"]:
                raise RuntimeError("비상정지로 중단됨")
            fmax = max(fmax, a["max_ferr"], b["max_ferr"])
            if self.status.get("error_code", 0):
                break
        errst = abs(self.status.get("actual_pos", 0) - origin)
        M.update({"err_stress": errst, "ferr_stress": fmax})
        self._t_item(res, "급가감속 후 복귀오차", errst <= lim["err_stress"],
                     f"{errst} pulse (허용 {lim['err_stress']})")
        self._t_ferr(res, "급가감속 최대 추종오차", fmax, lim["ferr_max"])
        self._chk_fault(res, "급가감속 구간 폴트")

        # ---- 9. 원점 복귀 + 마무리 -----------------------------------------
        step(9, "원점 복귀·마무리")
        back = origin - self.status.get("actual_pos", 0)
        if abs(back) > 5:
            self._test_move(back, 3.0, TEST_ACCEL_NORMAL)
        errf = abs(self.status.get("actual_pos", 0) - origin)
        M["err_final"] = errf
        self._t_item(res, "최종 원점 복귀오차", errf <= lim["err_final"],
                     f"{errf} pulse (허용 {lim['err_final']}) "
                     f"= {errf / PPR * 360:.2f}도")
        self._chk_fault(res, "최종 폴트")
        self._t_item(res, "목표도달 비트(6041 bit10)", True,
                     "관측됨" if M.get("bit10_seen") else "관측 안 됨(정지판정은 속도로 함)",
                     critical=False)

    def _chk_fault(self, res, label):
        ec = self.status.get("error_code", 0)
        self._t_item(res, label, ec == 0,
                     "없음" if ec == 0 else
                     f"0x{ec:04X} {ERROR_TEXT.get(ec, '알수없음')}")
        if ec:
            # 다음 구간을 위해 폴트 해제 시도
            self._fault_reset()
            self._enable()

    def _finish_selftest(self, res):
        """판정 → 로그 → CSV 한 줄 추가."""
        hard = [it for it in res["items"] if it["critical"] and not it["ok"]]
        warn = [it for it in res["items"] if not it["critical"] and not it["ok"]]
        verdict = "합격" if not hard else "불합격"
        res["verdict"] = verdict
        res["fail_names"] = ", ".join(it["name"] for it in hard)
        res["warn_names"] = ", ".join(it["name"] for it in warn)
        self.status["test_verdict"] = verdict
        self.status["test_fail_names"] = res["fail_names"]
        self.status["test_warn_names"] = res["warn_names"]
        mark = "✅ 합격" if verdict == "합격" else "❌ 불합격"
        self.log(f"===== [{res['id']}] {mark}  "
                 f"({res['meas'].get('elapsed_s')}초) =====")
        if hard:
            self.log(f"    불합격 항목: {res['fail_names']}")
        if warn:
            self.log(f"    경고 항목: {res['warn_names']}")
        self.log("    → 운전 OFF 상태. 랜선/전원 분리하고 다음 모터를 물리세요\n")
        try:
            self._write_csv(res)
            self.status["test_csv"] = TEST_CSV
        except Exception as e:  # noqa
            self.log(f"[ERR] 결과 CSV 저장 실패: {e}")

    CSV_COLS = ["시각", "아이디", "판정", "불합격항목", "경고항목",
                "시리얼", "제조사ID", "제품코드", "리비전", "장치명",
                "운전모드", "회전당펄스", "피크전류mA", "지원모드",
                "운전허가ms", "정지흔들림", "err_1rev", "ferr_1rev",
                "err_5rev", "ferr_5rev", "속도스윕", "err_sweep",
                "err_stress", "ferr_stress", "err_final", "TxPDO길이", "bit10",
                "소요초", "메모"]

    def _write_csv(self, res):
        os.makedirs(TEST_RESULT_DIR, exist_ok=True)
        M = res["meas"]

        def hexs(v):
            return f"0x{v:X}" if isinstance(v, int) else ""

        row = {
            "시각": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "아이디": res["id"], "판정": res["verdict"],
            "불합격항목": res["fail_names"], "경고항목": res["warn_names"],
            "시리얼": M.get("serial", ""), "제조사ID": hexs(M.get("man")),
            "제품코드": hexs(M.get("product")), "리비전": hexs(M.get("revision")),
            "장치명": M.get("name", ""), "운전모드": M.get("drive_mode", ""),
            "회전당펄스": M.get("ppr", ""), "피크전류mA": M.get("peak_ma", ""),
            "지원모드": hexs(M.get("sup_modes")), "운전허가ms": M.get("enable_ms", ""),
            "정지흔들림": M.get("idle_jitter", ""),
            "err_1rev": M.get("err_1rev", ""), "ferr_1rev": M.get("ferr_1rev", ""),
            "err_5rev": M.get("err_5rev", ""), "ferr_5rev": M.get("ferr_5rev", ""),
            "속도스윕": M.get("sweep", ""), "err_sweep": M.get("err_sweep", ""),
            "err_stress": M.get("err_stress", ""), "ferr_stress": M.get("ferr_stress", ""),
            "err_final": M.get("err_final", ""),
            "TxPDO길이": M.get("tx_len", ""),
            "bit10": "Y" if M.get("bit10_seen") else "N",
            "소요초": M.get("elapsed_s", ""),
            "메모": res.get("note", ""),
        }
        new = not os.path.exists(TEST_CSV)
        with open(TEST_CSV, "a", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=self.CSV_COLS)
            if new:
                w.writeheader()
            w.writerow(row)
        self.log(f"    💾 결과 저장: {TEST_CSV}")

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
    ACCEL = 100000                   # 가속 (firm한 출발, pulse/s^2) — 밀림 방지
    # 감속 프리셋(pulse/s^2). 낮을수록 천천히 멈춤=회생/전류 스파이크 완화.
    # 단, 너무 낮으면 목표 도달이 늘어져 밀림 → 너무 낮은 값은 두지 않음.
    # 이 드라이버는 S커브가 없어 사다리꼴뿐 → 부드러움은 이 값으로만 조절.
    DECEL_PRESETS = {"부드럽": 40000, "보통": 70000, "빠름": 120000}
    COL = {"idle": "#6b7280", "wait": "#2563eb", "run": "#16a34a", "fault": "#dc2626"}

    def __init__(self, root):
        self.root = root
        root.title("TL-E EtherCAT 모터 제어")
        root.geometry("1000x900")
        root.minsize(720, 460)

        self.log_q = queue.Queue()
        self.status = {"state": "미연결", "status_word": 0, "actual_pos": 0,
                       "actual_speed": 0, "error_code": 0, "slave_name": "-",
                       "wkc_ok": True, "brake_on": False, "current_ma": 0,
                       "hold_pct": 0, "hold_ma": 0, "io_status": 0,
                       "limit_pos": False, "limit_neg": False, "limit_home": False,
                       "di_raw": 0, "error_sdo": 0, "pos_dev": 0,
                       "follow_err": 0, "hardstop_hit": False,
                       # 새 모터 테스트
                       "test_running": False, "test_step": "", "test_items": [],
                       "test_verdict": "", "test_fail_names": "",
                       "test_warn_names": "", "test_id": "", "test_csv": ""}
        self.worker = None
        self._was_fault = False
        self._test_was_running = False
        self._test_items_n = -1        # 항목 수가 늘 때만 다시 그린다
        self._loading = False                          # 리밋 UI 동기화 중 콜백 억제
        self.v_speed_rev = tk.DoubleVar(value=1.0)     # rev/s
        self.v_step = tk.DoubleVar(value=5.0)          # deg
        self.v_current = tk.StringVar(value="3000")    # 전류 제한 mA
        self.v_hold = tk.StringVar(value="40")         # 정지 홀딩 전류 % (2307h)
        self.v_posdev = tk.StringVar(value="4000")     # 위치편차 임계 pulse (230Dh)
        # 리미트 스위치 설정 변수. 단자 라벨은 DI0~DI4(0-index), DIn → 2510h+n.
        # 기본: 정방향=DI3(2513h), 역방향=DI4(2514h) — 공장 기본 리밋 포트와 동일
        self.v_pos_en = tk.BooleanVar(value=False)
        self.v_pos_di = tk.StringVar(value="DI3")
        self.v_pos_nc = tk.StringVar(value="NO(A접점)")
        self.v_neg_en = tk.BooleanVar(value=False)
        self.v_neg_di = tk.StringVar(value="DI4")
        self.v_neg_nc = tk.StringVar(value="NO(A접점)")
        self.v_limit_mode = tk.StringVar(value="정지")
        self.v_decel = tk.StringVar(value="보통")       # 감속 프리셋 키
        self.v_hardstop_on = tk.BooleanVar(value=False)  # 끝단감지(위치편차) 사용
        self.v_hardstop_thr = tk.StringVar(value="3000") # 끝단감지 임계 pulse
        # --- 새 모터 테스트: 아이디 ---
        # 이 드라이버가 개체 시리얼을 갖고 있는지는 물려봐야 안다(1018h:04).
        # 그래서 아이디의 기준은 사람이 붙이는 번호로 두고, 하드웨어 시리얼은
        # 읽히면 같이 기록한다. 로트 + 일련번호 → 예: L2608-001
        self.v_lot = tk.StringVar(value=datetime.date.today().strftime("L%y%m"))
        self.v_seq = tk.StringVar(value="1")
        self.v_total = tk.StringVar(value="42")
        self.v_autoinc = tk.BooleanVar(value=True)
        self.v_note = tk.StringVar(value="")
        self._test_last_id = ""

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

        # ===== 비상정지 (항상 보이는 큰 빨강 버튼 · Esc키도 동작) =====
        self.btn_estop = tk.Button(root, text="🛑 비상정지 (E-STOP · Esc)",
                                   bg="#dc2626", fg="white", activebackground="#b91c1c",
                                   activeforeground="white", font=("Helvetica", 15, "bold"),
                                   relief="raised", bd=3, command=self.on_estop)
        self.btn_estop.pack(fill="x", padx=10, pady=(6, 0))
        self.root.bind("<Escape>", lambda e: self.on_estop())

        # 2컬럼: 좌(컨트롤, 스크롤) | 우(상세·로그). 가운데 경계를 드래그해 폭 조절.
        main = ttk.PanedWindow(root, orient="horizontal")
        main.pack(fill="both", expand=True)

        left = ttk.Frame(main)
        canvas = tk.Canvas(left, highlightthickness=0, width=520)
        vsb = ttk.Scrollbar(left, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        body = ttk.Frame(canvas, padding=(10, 8))
        body_id = canvas.create_window((0, 0), window=body, anchor="nw")
        canvas.bind("<Configure>", lambda e: canvas.itemconfigure(body_id, width=e.width))
        body.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        # 맥은 delta가 작아서 나눗셈하면 0 → 부호 기반으로 스크롤
        canvas.bind_all("<MouseWheel>",
                        lambda e: canvas.yview_scroll(-1 if e.delta > 0 else 1, "units"))

        right = ttk.Frame(main)
        self._build_logpane(right)

        main.add(left, weight=0)
        main.add(right, weight=1)
        # 초기 경계 위치(드래그로 변경 가능)
        self.root.after(120, lambda: main.sashpos(0, 560))
        # 패널이 길어져서 시작하자마자 아래로 밀려 보이는 일이 있어 맨 위로 맞춘다
        self.root.after(200, lambda: canvas.yview_moveto(0))

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

        # ===== ⓪ 새 모터 테스트 (신품 입고검사) =====
        ft = ttk.LabelFrame(body, text=" 🧪 새 모터 테스트 (신품 입고검사) ")
        ft.pack(fill="x", pady=4)

        # 아이디: 로트 + 일련번호. 하드웨어 시리얼은 검사 중에 읽어서 같이 기록한다.
        ti = ttk.Frame(ft); ti.pack(fill="x", padx=8, pady=(8, 2))
        ttk.Label(ti, text="로트").pack(side="left")
        ttk.Entry(ti, textvariable=self.v_lot, width=7).pack(side="left", padx=(3, 8))
        ttk.Label(ti, text="번호").pack(side="left")
        ttk.Entry(ti, textvariable=self.v_seq, width=5).pack(side="left", padx=3)
        ttk.Label(ti, text="/").pack(side="left")
        ttk.Entry(ti, textvariable=self.v_total, width=4).pack(side="left", padx=3)
        ttk.Checkbutton(ti, text="자동증가", variable=self.v_autoinc).pack(side="left", padx=6)

        ti2 = ttk.Frame(ft); ti2.pack(fill="x", padx=8, pady=2)
        ttk.Label(ti2, text="아이디").pack(side="left")
        self.lbl_testid = tk.Label(ti2, text="-", font=("Menlo", 13, "bold"), fg="#111827")
        self.lbl_testid.pack(side="left", padx=6)
        ttk.Label(ti2, text="메모").pack(side="left", padx=(10, 2))
        ttk.Entry(ti2, textvariable=self.v_note, width=16).pack(side="left", padx=2)
        for v in (self.v_lot, self.v_seq):
            v.trace_add("write", lambda *a: self._sync_testid())

        self.btn_test = tk.Button(ft, text="🧪 새 모터 테스트 시작",
                                  bg="#2563eb", fg="white", activebackground="#1d4ed8",
                                  activeforeground="white",
                                  font=("Helvetica", 15, "bold"), relief="raised", bd=3,
                                  command=self.on_selftest)
        self.btn_test.pack(fill="x", padx=8, pady=(6, 2))

        self.lbl_tprog = ttk.Label(ft, text="대기 중 — 연결 후 시작을 누르세요",
                                   font=("Helvetica", 11))
        self.lbl_tprog.pack(anchor="w", padx=10, pady=(0, 2))

        self.lbl_tverdict = tk.Label(ft, text="판정 —", bg="#e5e7eb", fg="#111827",
                                     font=("Helvetica", 16, "bold"), height=2)
        self.lbl_tverdict.pack(fill="x", padx=8, pady=2)

        self.txt_titems = tk.Text(ft, height=9, font=("Menlo", 10), wrap="none")
        self.txt_titems.pack(fill="x", padx=8, pady=2)

        tb = ttk.Frame(ft); tb.pack(fill="x", padx=8, pady=(0, 8))
        self.lbl_tcount = ttk.Label(tb, text="합격 0 · 불합격 0 · 남음 42")
        self.lbl_tcount.pack(side="left")
        ttk.Button(tb, text="결과 CSV 열기",
                   command=self.on_open_csv).pack(side="right", padx=2)
        ttk.Button(tb, text="폴더 열기",
                   command=self.on_open_dir).pack(side="right", padx=2)

        self._sync_testid()
        self._refresh_test_count()

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
        r4 = ttk.Frame(f2); r4.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Label(r4, text="정지 홀딩(%)").pack(side="left")
        ttk.Entry(r4, textvariable=self.v_hold, width=6).pack(side="left", padx=4)
        for v in ("40", "60", "80", "100"):
            ttk.Button(r4, text=v, width=4, command=lambda x=v: self.v_hold.set(x)).pack(side="left", padx=1)
        ttk.Button(r4, text="적용", command=self.on_holdcur).pack(side="left", padx=4)
        r5 = ttk.Frame(f2); r5.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Label(r5, text="위치편차 허용(pulse)").pack(side="left")
        ttk.Entry(r5, textvariable=self.v_posdev, width=7).pack(side="left", padx=4)
        for v in ("4000", "10000", "20000", "50000"):
            ttk.Button(r5, text=v, width=5, command=lambda x=v: self.v_posdev.set(x)).pack(side="left", padx=1)
        ttk.Button(r5, text="적용", command=self.on_posdev).pack(side="left", padx=4)

        # ===== ③ 이동 =====
        f3 = ttk.LabelFrame(body, text=" ③ 이동 (조그) ")
        f3.pack(fill="x", pady=4)
        sp = ttk.Frame(f3); sp.pack(fill="x", padx=8, pady=(8, 2))
        ttk.Label(sp, text="속도").pack(side="left")
        ttk.Scale(sp, from_=0.1, to=25.0, variable=self.v_speed_rev, orient="horizontal",
                  command=self._on_speed).pack(side="left", expand=True, fill="x", padx=6)
        self.lbl_speed = ttk.Label(sp, text="1.0 rev/s", width=10)
        self.lbl_speed.pack(side="left")

        spb = ttk.Frame(f3); spb.pack(fill="x", padx=8, pady=(0, 2))
        ttk.Label(spb, text="빠른설정(rev/s)").pack(side="left")
        for v in (1, 3, 5, 10, 15, 25):
            ttk.Button(spb, text=str(v), width=3,
                       command=lambda x=v: self._set_speed(x)).pack(side="left", padx=1)

        # 감속(정지 부드럽기). 낮을수록 회생/전류 스파이크↓, 너무 낮으면 밀림.
        dc = ttk.Frame(f3); dc.pack(fill="x", padx=8, pady=(0, 2))
        ttk.Label(dc, text="감속(정지)").pack(side="left")
        ttk.Combobox(dc, textvariable=self.v_decel, values=list(self.DECEL_PRESETS.keys()),
                     width=7, state="readonly").pack(side="left", padx=4)
        self.v_decel.trace_add("write", lambda *a: self._push_config())
        ttk.Label(dc, text="← 폴트나면 낮춤 / 밀리면 높임").pack(side="left", padx=4)

        stp = ttk.Frame(f3); stp.pack(fill="x", padx=8, pady=2)
        ttk.Label(stp, text="스텝").pack(side="left")
        for lbl, val in [("1°", 1), ("5°", 5), ("15°", 15), ("90°", 90), ("360°", 360)]:
            ttk.Button(stp, text=lbl, width=4, command=lambda v=val: self.v_step.set(v)).pack(side="left", padx=2)
        self.lbl_step = ttk.Label(stp, text="5°", width=6)
        self.lbl_step.pack(side="left", padx=4)

        # 스텝 조그(정해진 각도만큼 이동). 역방향=샤프트 나감 / 정방향=샤프트 들어옴
        jog = ttk.Frame(f3); jog.pack(fill="x", padx=8, pady=(4, 2))
        ttk.Button(jog, text="◀  나감(역)", style="Big.TButton", command=lambda: self.on_step(-1)).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(jog, text="들어옴(정)  ▶", style="Big.TButton", command=lambda: self.on_step(1)).pack(side="left", expand=True, fill="x", padx=3)

        # 연속 조그(버튼을 누르는 동안 계속 이동, 떼면 정지)
        hjog = ttk.Frame(f3); hjog.pack(fill="x", padx=8, pady=(0, 10))
        b_out = ttk.Button(hjog, text="◀◀ 계속 나감", style="Big.TButton")
        b_out.pack(side="left", expand=True, fill="x", padx=3)
        b_in = ttk.Button(hjog, text="계속 들어옴 ▶▶", style="Big.TButton")
        b_in.pack(side="left", expand=True, fill="x", padx=3)
        b_out.bind("<ButtonPress-1>", lambda e: self.on_jog_start(-1))
        b_out.bind("<ButtonRelease-1>", lambda e: self.on_jog_stop())
        b_in.bind("<ButtonPress-1>", lambda e: self.on_jog_start(1))
        b_in.bind("<ButtonRelease-1>", lambda e: self.on_jog_stop())

        # ===== ④ 리미트 스위치 =====
        f_lim = ttk.LabelFrame(body, text=" ④ 리미트 스위치 ")
        f_lim.pack(fill="x", pady=4)
        DI_VALUES = ["DI0", "DI1", "DI2", "DI3", "DI4"]   # 단자 인쇄와 동일, DIn→2510h+n
        NC_VALUES = ["NO(A접점)", "NC(B접점)"]

        lp = ttk.Frame(f_lim); lp.pack(fill="x", padx=8, pady=(8, 2))
        ttk.Checkbutton(lp, text="정방향(+)", variable=self.v_pos_en,
                        command=lambda: self._apply_limit("pos")).pack(side="left")
        ttk.Combobox(lp, textvariable=self.v_pos_di, values=DI_VALUES, width=4,
                     state="readonly").pack(side="left", padx=3)
        ttk.Combobox(lp, textvariable=self.v_pos_nc, values=NC_VALUES, width=10,
                     state="readonly").pack(side="left", padx=3)
        self.led_pos = tk.Label(lp, text="●", fg="#9ca3af", font=("Helvetica", 14))
        self.led_pos.pack(side="left", padx=4)

        ln = ttk.Frame(f_lim); ln.pack(fill="x", padx=8, pady=2)
        ttk.Checkbutton(ln, text="역방향(−)", variable=self.v_neg_en,
                        command=lambda: self._apply_limit("neg")).pack(side="left")
        ttk.Combobox(ln, textvariable=self.v_neg_di, values=DI_VALUES, width=4,
                     state="readonly").pack(side="left", padx=3)
        ttk.Combobox(ln, textvariable=self.v_neg_nc, values=NC_VALUES, width=10,
                     state="readonly").pack(side="left", padx=3)
        self.led_neg = tk.Label(ln, text="●", fg="#9ca3af", font=("Helvetica", 14))
        self.led_neg.pack(side="left", padx=4)

        lm = ttk.Frame(f_lim); lm.pack(fill="x", padx=8, pady=(2, 8))
        ttk.Label(lm, text="리밋 동작").pack(side="left")
        ttk.Combobox(lm, textvariable=self.v_limit_mode, values=["정지", "급정지", "무동작"],
                     width=6, state="readonly").pack(side="left", padx=3)
        ttk.Button(lm, text="동작적용", command=self.on_limit_mode).pack(side="left", padx=2)
        ttk.Button(lm, text="💾 저장", command=lambda: self._post("save_params")).pack(side="left", padx=2)

        # 원시 DI 입력상태(2100h) — 배선/센서 진단용. 기능배정·저장과 무관하게
        # 단자에 신호가 들어오면 해당 자리가 1로 바뀐다(센서 앞에 금속 대보며 확인).
        ld = ttk.Frame(f_lim); ld.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Label(ld, text="원시 DI(2100h)").pack(side="left")
        self.lbl_di = ttk.Label(ld, text="DI0-4  - - - - -", font=("Menlo", 11))
        self.lbl_di.pack(side="left", padx=6)

        # DI/NC 콤보 변경 시 즉시 재적용 (초기값 설정 뒤에 trace 연결 → 시작 시 오발동 방지)
        self.v_pos_di.trace_add("write", lambda *a: self._apply_limit("pos"))
        self.v_pos_nc.trace_add("write", lambda *a: self._apply_limit("pos"))
        self.v_neg_di.trace_add("write", lambda *a: self._apply_limit("neg"))
        self.v_neg_nc.trace_add("write", lambda *a: self._apply_limit("neg"))

        # ===== ⑤ 끝단감지 (위치편차 · 리미트 스위치 대체) =====
        f_hs = ttk.LabelFrame(body, text=" ⑤ 끝단감지 (위치편차) ")
        f_hs.pack(fill="x", pady=4)
        hr = ttk.Frame(f_hs); hr.pack(fill="x", padx=8, pady=(8, 2))
        ttk.Checkbutton(hr, text="사용", variable=self.v_hardstop_on,
                        command=self.on_hardstop).pack(side="left")
        ttk.Label(hr, text="임계(pulse)").pack(side="left", padx=(8, 2))
        ttk.Entry(hr, textvariable=self.v_hardstop_thr, width=7).pack(side="left")
        for v in ("2000", "3000", "5000"):
            ttk.Button(hr, text=v, width=5,
                       command=lambda x=v: self.v_hardstop_thr.set(x)).pack(side="left", padx=1)
        ttk.Button(hr, text="적용", command=self.on_hardstop).pack(side="left", padx=4)
        hr2 = ttk.Frame(f_hs); hr2.pack(fill="x", padx=8, pady=(0, 8))
        self.lbl_ferr = ttk.Label(hr2, text="추종오차 0", font=("Menlo", 11))
        self.lbl_ferr.pack(side="left")
        ttk.Label(hr2, text=" (감지 시 정지·홀딩유지, 하중 안떨어짐. 폴트 아님)",
                  font=("Helvetica", 9)).pack(side="left")

        if not PYSOEM_OK:
            self._log(f"[ERR] pysoem 로드 실패: {PYSOEM_ERR}  → pip3 install pysoem")

        self.root.after(150, self._poll)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_logpane(self, parent):
        """오른쪽 컬럼: 상태 요약 + 로그."""
        f4 = ttk.LabelFrame(parent, text=" 상세 · 로그 ")
        f4.pack(fill="both", expand=True, padx=(4, 8), pady=8)
        self.m_detail = ttk.Label(f4, text="StatusWord ----", font=("Menlo", 11),
                                  justify="left")
        self.m_detail.pack(anchor="w", padx=8, pady=(8, 4))
        self.log = tk.Text(f4, width=46, font=("Menlo", 10))
        self.log.pack(fill="both", expand=True, padx=8, pady=(0, 8))

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

    def _decel_val(self):
        return self.DECEL_PRESETS.get(self.v_decel.get(), 70000)

    def _push_config(self):
        if self.worker is not None:
            self._post("config", self._speed_pulse(), self.ACCEL, self._decel_val())

    # ---- 콜백 ----
    def _on_speed(self, _=None):
        self.lbl_speed.config(text=f"{self.v_speed_rev.get():.1f} rev/s")
        self._push_config()

    def _set_speed(self, rev):
        self.v_speed_rev.set(float(rev))
        self._on_speed()

    def on_estop(self, *_):
        """비상정지: 워커에 직접(큐 우회) 전달해 즉시 토크 차단."""
        self._log("[🛑] 비상정지 요청")
        if self.worker is not None:
            self.worker.trigger_estop()

    def on_jog_start(self, direction):
        big = int(1000 * self.PPR)     # 사실상 무한(1000회전) 상대이동
        self._log(f"[i] 연속조그 {'들어옴(정)' if direction > 0 else '나감(역)'} — 누르는 동안 이동")
        self._post("jog_start", direction, big)

    def on_jog_stop(self, *_):
        self._post("jog_stop")

    def on_connect(self):
        if not PYSOEM_OK:
            self._log("[ERR] pysoem 없음"); return
        name = self.iface.get().strip()
        if not name:
            self._log("[!] 인터페이스명을 입력하세요 (예: en7)"); return
        self._ensure_worker()
        self.worker.post("connect", name)
        self.worker.post("config", self._speed_pulse(), self.ACCEL, self._decel_val())
        self.worker.post("current", None)      # 현재 전류제한 읽기
        self.worker.post("holdcur", None)      # 현재 정지 홀딩전류 읽기
        self.worker.post("posdev", None)       # 현재 위치편차 임계 읽기
        self.worker.post("limit_read")         # 현재 리미트/DI 설정 읽어 UI 동기화
        self.on_hardstop()                     # 끝단감지 설정 워커에 반영

    def on_disconnect(self):
        self._post("disconnect")

    def on_current(self):
        try:
            ma = max(0, min(6000, int(float(self.v_current.get()))))
        except Exception:
            self._log("[!] 전류값 숫자 오류"); return
        self._post("current", ma)

    def on_holdcur(self):
        try:
            pct = max(0, min(100, int(float(self.v_hold.get()))))
        except Exception:
            self._log("[!] 홀딩% 숫자 오류"); return
        self._post("holdcur", pct)

    def on_posdev(self):
        try:
            v = max(100, min(65535, int(float(self.v_posdev.get()))))
        except Exception:
            self._log("[!] 위치편차 숫자 오류"); return
        self._post("posdev", v)

    def on_hardstop(self):
        try:
            thr = max(200, int(float(self.v_hardstop_thr.get())))
        except Exception:
            self._log("[!] 끝단감지 임계 숫자 오류"); return
        self._post("hardstop", self.v_hardstop_on.get(), thr)

    # ---- 새 모터 테스트 콜백 ----
    def _test_id(self):
        """아이디 = 로트 + 3자리 일련번호. 예: L2608-001"""
        lot = self.v_lot.get().strip() or "L"
        try:
            n = int(float(self.v_seq.get()))
        except Exception:
            n = 0
        return f"{lot}-{n:03d}"

    def _sync_testid(self, *_):
        self.lbl_testid.config(text=self._test_id())

    def on_selftest(self):
        if self.worker is None:
            self._log("[!] 먼저 ① 연결하세요"); return
        if self.status.get("test_running"):
            self._log("[!] 이미 검사 중입니다"); return
        mid = self._test_id()
        self._test_items_n = -1
        self.txt_titems.delete("1.0", "end")
        self.lbl_tverdict.config(text="검사 중…", bg="#f59e0b", fg="white")
        self.btn_test.config(state="disabled", text="검사 중…")
        self._post("selftest", mid, self.v_note.get().strip())

    def _render_test_items(self, items):
        if len(items) == self._test_items_n:
            return
        self._test_items_n = len(items)
        self.txt_titems.delete("1.0", "end")
        for it in items:
            mark = "OK  " if it["ok"] else ("FAIL" if it["critical"] else "warn")
            self.txt_titems.insert("end", f"{mark}  {it['name']}: {it['detail']}\n")
        self.txt_titems.see("end")

    def _refresh_test_count(self):
        """CSV를 읽어 합격/불합격/남은 수량 표시."""
        ok = ng = 0
        try:
            if os.path.exists(TEST_CSV):
                with open(TEST_CSV, encoding="utf-8-sig") as f:
                    for r in csv.DictReader(f):
                        if r.get("판정") == "합격":
                            ok += 1
                        elif r.get("판정") == "불합격":
                            ng += 1
        except Exception:
            pass
        try:
            total = int(float(self.v_total.get()))
        except Exception:
            total = 0
        self.lbl_tcount.config(text=f"합격 {ok} · 불합격 {ng} · "
                                    f"남음 {max(0, total - ok - ng)}")

    def _open_path(self, path):
        """맥에서 파일/폴더 열기. sudo로 돌고 있으면 원래 사용자 권한으로 연다."""
        if not os.path.exists(path):
            self._log(f"[!] 아직 없습니다: {path}"); return
        user = os.environ.get("SUDO_USER")
        cmd = ["sudo", "-u", user, "open", path] if user else ["open", path]
        try:
            subprocess.Popen(cmd)
        except Exception as e:  # noqa
            self._log(f"[!] 열기 실패: {e}  → 직접 여세요: {path}")

    def on_open_csv(self):
        self._open_path(TEST_CSV)

    def on_open_dir(self):
        self._open_path(TEST_RESULT_DIR)

    # ---- 리미트 스위치 콜백 ----
    def _apply_limit(self, side):
        """체크박스/DI/NC 변경 시 해당 방향 리밋 설정을 워커에 전달."""
        if self._loading or self.worker is None:
            return
        di_var = self.v_pos_di if side == "pos" else self.v_neg_di
        en_var = self.v_pos_en if side == "pos" else self.v_neg_en
        nc_var = self.v_pos_nc if side == "pos" else self.v_neg_nc
        ch = int(di_var.get()[2:])                       # "DI3" -> 3 (0-index 채널)
        nc = nc_var.get().startswith("NC")
        self._post("set_limit", side, ch, en_var.get(), nc)

    def on_limit_mode(self, *_):
        m = {"정지": 0, "급정지": 1, "무동작": 2}.get(self.v_limit_mode.get(), 0)
        self._post("limit_mode", m)

    def _sync_limit_ui(self, cfg):
        """드라이버에서 읽은 실제 설정으로 UI를 맞춤(콜백 재발동 억제)."""
        funcs = cfg.get("funcs", []); lvl = cfg.get("level", 0); mode = cfg.get("mode", 0)
        self._loading = True
        try:
            pos_ch = next((i for i, f in enumerate(funcs) if f == 2), None)
            neg_ch = next((i for i, f in enumerate(funcs) if f == 3), None)
            if pos_ch is not None:
                self.v_pos_di.set(f"DI{pos_ch}"); self.v_pos_en.set(True)
                self.v_pos_nc.set("NC(B접점)" if (lvl >> pos_ch) & 1 else "NO(A접점)")
            else:
                self.v_pos_en.set(False)
            if neg_ch is not None:
                self.v_neg_di.set(f"DI{neg_ch}"); self.v_neg_en.set(True)
                self.v_neg_nc.set("NC(B접점)" if (lvl >> neg_ch) & 1 else "NO(A접점)")
            else:
                self.v_neg_en.set(False)
            self.v_limit_mode.set({0: "정지", 1: "급정지", 2: "무동작"}.get(mode, "정지"))
        finally:
            self._loading = False

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
        limit_active = bool(sw & 0x0800)   # bit11 internal limit active
        # 에러코드: PDO값 우선, 0이면 SDO 교차확인값 사용
        ec = s["error_code"] or s.get("error_sdo", 0)

        # 폴트 발생 순간 경고. 603F 코드가 있으면 그 원인, 없으면(=리밋/정지류) 안내 구분.
        if fault and not self._was_fault:
            if ec:
                self._log(f"[⚠️폴트] 모터 정지! 에러 0x{ec:04X} "
                          f"{ERROR_TEXT.get(ec, '알수없음')} → 원인 제거 후 '알람리셋'")
            else:
                self._log("[⚠️정지] 폴트비트 ON인데 603F 에러코드 없음 "
                          f"(SW 0x{sw:04X}{', 리밋활성' if limit_active else ''}). "
                          "리밋/급정지류 정지일 수 있음 → 반대로 빠져나오거나 알람리셋")
        self._was_fault = fault

        # 리밋 설정 읽기 완료 시 UI 1회 동기화
        if self.status.get("limit_cfg_dirty"):
            self.status["limit_cfg_dirty"] = False
            self._sync_limit_ui(self.status.get("limit_cfg", {}))
        # 위치편차 임계 읽기 → 엔트리 1회 반영
        if self.status.get("pos_dev_sync"):
            self.status["pos_dev_sync"] = False
            self.v_posdev.set(str(self.status.get("pos_dev", 4000)))

        # ---- 새 모터 테스트 표시 ----
        running = bool(s.get("test_running"))
        self._render_test_items(s.get("test_items", []))
        if running:
            self.lbl_tprog.config(text=f"진행 중 — {s.get('test_step', '')}")
        if self._test_was_running and not running:
            # 검사 1대가 막 끝난 순간: 판정 표시 + 번호 자동증가 + 수량 갱신
            v = s.get("test_verdict", "")
            if v == "합격":
                self.lbl_tverdict.config(text=f"✅ 합격  [{s.get('test_id', '')}]",
                                         bg="#16a34a", fg="white")
            elif v == "불합격":
                self.lbl_tverdict.config(
                    text=f"❌ 불합격  [{s.get('test_id', '')}]\n{s.get('test_fail_names', '')}",
                    bg="#dc2626", fg="white")
            else:
                self.lbl_tverdict.config(text="중단됨", bg="#6b7280", fg="white")
            self.lbl_tprog.config(text="끝. 랜선/전원 분리하고 다음 모터를 물리세요")
            self.btn_test.config(state="normal", text="🧪 새 모터 테스트 시작")
            if self.v_autoinc.get():
                try:
                    self.v_seq.set(str(int(float(self.v_seq.get())) + 1))
                except Exception:
                    pass
            self._refresh_test_count()
        self._test_was_running = running

        # 리밋 LED: 빨강=신호감지(트리거) / 초록=사용중(정상) / 회색=미사용
        self.led_pos.config(fg="#dc2626" if s.get("limit_pos")
                            else ("#16a34a" if self.v_pos_en.get() else "#9ca3af"))
        self.led_neg.config(fg="#dc2626" if s.get("limit_neg")
                            else ("#16a34a" if self.v_neg_en.get() else "#9ca3af"))

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
        errtxt = (ERROR_TEXT.get(ec, "알수없음") if ec
                  else ("리밋/정지" if fault else "없음"))
        rev = s["actual_pos"] / self.PPR
        self.b_info.config(bg=col,
                           text=f"위치 {rev:+.2f} 회전   ·   속도 {s['actual_speed']:+d}   ·   에러 {errtxt}")

        self.lbl_step.config(text=f"{self.v_step.get():g}°")
        di = s.get("di_raw", 0)
        di_cells = "  ".join(f"{ch}:{'■' if (di >> ch) & 1 else '·'}" for ch in range(5))
        self.lbl_di.config(text=f"DI  {di_cells}")
        ferr = s.get("follow_err", 0)
        hit = s.get("hardstop_hit")
        self.lbl_ferr.config(text=f"추종오차 {ferr:+d}{'  🛑끝단감지!' if hit else ''}",
                             foreground="#dc2626" if hit else "")

        # 상태워드 비트 디코드 (에러코드가 0이어도 '왜' 멈췄는지 보이게)
        flags = []
        if sw & 0x0004: flags.append("운전허가")
        if sw & 0x0008: flags.append("폴트")
        if sw & 0x0080: flags.append("경고")
        if sw & 0x0400: flags.append("목표도달")
        if sw & 0x0800: flags.append("리밋활성")
        flags_txt = " ".join(flags) if flags else "-"
        brake = "🔓해제" if s.get("brake_on") else "🔒잠금"
        lim = []
        if s.get("limit_pos"): lim.append("+리밋")
        if s.get("limit_neg"): lim.append("−리밋")
        if s.get("limit_home"): lim.append("홈")
        lim_txt = "⚠" + "/".join(lim) if lim else "정상"
        self.m_detail.config(
            text=(f"SW  0x{sw:04X}  [{flags_txt}]   WKC {'OK' if s['wkc_ok'] else 'BAD'}\n"
                  f"에러  PDO 0x{s['error_code']:04X}  SDO 0x{s.get('error_sdo', 0):04X}  "
                  f"{ERROR_TEXT.get(ec, '') if ec else ''}\n"
                  f"원시 DI  {di_cells}\n"
                  f"추종오차 {ferr:+d} pulse{'  🛑끝단' if hit else ''}\n"
                  f"기능리밋 {lim_txt}   브레이크 {brake}\n"
                  f"전류제한 {s.get('current_ma', 0)}mA  홀딩 {s.get('hold_pct', 0)}%"
                  f"(≈{s.get('hold_ma', 0)}mA)\n"
                  f"위치 {s['actual_pos']} pulse ({rev:+.2f}rev)"))

        self.root.after(150, self._poll)


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
