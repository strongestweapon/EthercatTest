#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""모터가 안 움직이는 원인 추적용 디버그 스크립트.
각 단계마다 StatusWord/위치를 전부 출력한다. 작은 각도(기본 5°)만 이동.
사용:  python3 debug_move.py [iface=en7] [step_deg=5]
"""
import sys
import struct
import time
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
STEP_DEG = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
PPR = 10000
SPEED = 5000
ACCEL = 50000
STEP = int(round(STEP_DEG / 360.0 * PPR))

master = pysoem.Master()
sl = None


def dec(sw):
    b = [("RDY", 1), ("ON", 2), ("ENA", 4), ("FLT", 8), ("VOLT", 0x10),
         ("QS", 0x20), ("REM", 0x200), ("TRGT", 0x400), ("ACK", 0x1000)]
    return f"0x{sw:04X} [" + " ".join(n for n, m in b if sw & m) + "]"


def io(cw, target):
    sl.output = struct.pack("<HiI", cw & 0xFFFF, target, SPEED)
    master.send_processdata()
    wkc = master.receive_processdata(2000)
    sw, apos, aspd = struct.unpack("<Hii", bytes(sl.input)[:10])
    return sw, apos, aspd, wkc


def pump(cw, target, n):
    r = (0, 0, 0, 0)
    for _ in range(n):
        r = io(cw, target)
        time.sleep(0.004)
    return r


def sdo_u16(idx, sub=0):
    return struct.unpack("<H", sl.sdo_read(idx, sub)[:2])[0]


def cfg(pos):
    s = master.slaves[pos]
    # RxPDO: 6040(16) 607A(32) 6081(32)
    s.sdo_write(0x1C12, 0, struct.pack("B", 0))
    s.sdo_write(0x1600, 0, struct.pack("B", 0))
    s.sdo_write(0x1600, 1, struct.pack("<I", 0x60400010))
    s.sdo_write(0x1600, 2, struct.pack("<I", 0x607A0020))
    s.sdo_write(0x1600, 3, struct.pack("<I", 0x60810020))
    s.sdo_write(0x1600, 0, struct.pack("B", 3))
    s.sdo_write(0x1C12, 1, struct.pack("<H", 0x1600))
    s.sdo_write(0x1C12, 0, struct.pack("B", 1))
    # TxPDO: 6041(16) 6064(32) 606C(32)
    s.sdo_write(0x1C13, 0, struct.pack("B", 0))
    s.sdo_write(0x1A00, 0, struct.pack("B", 0))
    s.sdo_write(0x1A00, 1, struct.pack("<I", 0x60410010))
    s.sdo_write(0x1A00, 2, struct.pack("<I", 0x60640020))
    s.sdo_write(0x1A00, 3, struct.pack("<I", 0x606C0020))
    s.sdo_write(0x1A00, 0, struct.pack("B", 3))
    s.sdo_write(0x1C13, 1, struct.pack("<H", 0x1A00))
    s.sdo_write(0x1C13, 0, struct.pack("B", 1))
    # PP 모드 + 가감속/속도
    s.sdo_write(0x6060, 0, struct.pack("b", 1))
    s.sdo_write(0x6083, 0, struct.pack("<I", ACCEL))
    s.sdo_write(0x6084, 0, struct.pack("<I", ACCEL))
    s.sdo_write(0x6081, 0, struct.pack("<I", SPEED))


def main():
    global sl
    print(f"[i] iface={IFACE} step={STEP_DEG}° ({STEP} pulse) speed={SPEED}")
    master.open(IFACE)
    n = master.config_init()
    print(f"[i] config_init → 슬레이브 {n}개")
    if n <= 0:
        return
    sl = master.slaves[0]
    sl.config_func = cfg
    master.config_map()
    st = master.state_check(pysoem.SAFEOP_STATE, 50000)
    print(f"[i] SAFEOP check → {hex(st)}")
    # OP 진입
    sl.output = struct.pack("<HiI", 0, 0, SPEED)
    master.send_processdata(); master.receive_processdata(2000)
    master.state = pysoem.OP_STATE
    master.write_state()
    for _ in range(300):
        master.send_processdata(); master.receive_processdata(2000)
        if master.state_check(pysoem.OP_STATE, 5000) == pysoem.OP_STATE:
            break
        time.sleep(0.002)
    print(f"[i] OP check → {hex(master.state_check(pysoem.OP_STATE, 5000))}")

    # 진단 정보
    print(f"[i] 6060h(설정모드)={sl.sdo_read(0x6060,0)[0]}  6061h(현재모드)={sl.sdo_read(0x6061,0)[0]}  (1=PP 기대)")
    print(f"[i] 6081h(속도)={sdo_u16(0x6081)}  603Fh(에러)=0x{sdo_u16(0x603F):04X}")

    sw, apos, aspd, wkc = io(0x00, 0)
    print(f"[초기]   {dec(sw)} pos={apos} wkc={wkc}")

    # 혹시 모를 폴트 리셋
    pump(0x80, 0, 10); pump(0x00, 0, 5)

    # Enable 시퀀스
    for cw in (0x06, 0x07, 0x0F):
        sw, apos, aspd, wkc = pump(cw, 0, 20)
        print(f"[CW={cw:#04x}] {dec(sw)} pos={apos} wkc={wkc}")

    if not (sw & 0x0004):
        print("[!] ENA 비트 안 켜짐 → 운전허가 실패. 603Fh 에러=0x%04X" % sdo_u16(0x603F))
        # 그래도 이동 시도는 안 함
        cleanup(); return
    print("[i] ✅ 운전허가(Operation Enabled) 됨. 이동 시도.")

    # 상대이동: base=0x0F|0x40(relative), trigger +0x10(bit4)
    base = 0x0F | 0x40
    pump(base, STEP, 3)
    print(f"[trig전] {dec(io(base, STEP)[0])}")
    pump(base | 0x10, STEP, 3)          # 0x5F 상승엣지
    print(f"[trig!]  {dec(io(base | 0x10, STEP)[0])}  (bit4 상승엣지로 신규위치 트리거)")

    # 이동 관찰 (~1.5초) — bit4 내려서 유지
    print("[i] 이동 관찰:")
    start = None
    for i in range(75):
        sw, apos, aspd, wkc = io(base, STEP)
        if start is None:
            start = apos
        if i % 5 == 0:
            print(f"   t={i*0.02:4.2f}s  {dec(sw)}  pos={apos:+6d}  spd={aspd:+6d}")
        time.sleep(0.02)
    moved = apos - start
    print(f"[결과] 시작pos={start} 끝pos={apos} 이동량={moved:+d} pulse "
          f"(목표 {STEP:+d})  → {'✅ 움직임' if abs(moved) > 5 else '❌ 안 움직임'}")
    print(f"[i] 최종 603Fh 에러=0x{sdo_u16(0x603F):04X}")

    cleanup()


def cleanup():
    try:
        pump(0x00, 0, 5)                 # disable
        master.state = pysoem.INIT_STATE
        master.write_state()
    except Exception:
        pass
    master.close()
    print("[i] 종료")


if __name__ == "__main__":
    main()
