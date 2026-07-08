#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Enable(운전허가) 전이 실패 원인 추적. 입력/리미트/모드/전이를 전부 실측.
모터는 안 움직임 (Enable까지만, 성공 시에만 작은 이동 옵션)."""
import sys
import struct
import time
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
DO_MOVE = "--move" in sys.argv
PPR = 10000
SPEED = 5000
ACCEL = 50000
STEP = int(round(5.0 / 360.0 * PPR))

master = pysoem.Master()
sl = None


def dec(sw):
    b = [("RDY", 1), ("ON", 2), ("ENA", 4), ("FLT", 8), ("VOLT", 0x10),
         ("QS", 0x20), ("NRDY", 0x40), ("REM", 0x200), ("TRGT", 0x400), ("ACK", 0x1000)]
    return f"0x{sw:04X}[" + " ".join(n for n, m in b if sw & m) + "]"


def dec_fd(v):
    b = [("NegLim", 1 << 0), ("PosLim", 1 << 1), ("Home", 1 << 2),
         ("Probe1", 1 << 12), ("Probe2", 1 << 13)]
    return f"0x{v & 0xFFFF:04X}[" + " ".join(n for n, m in b if v & m) + "]"


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


def u(idx, sub=0, fmt="<H"):
    try:
        raw = sl.sdo_read(idx, sub)
        return struct.unpack(fmt, raw[:struct.calcsize(fmt)])[0]
    except Exception as e:
        return f"(err:{e})"


def cfg(pos):
    s = master.slaves[pos]
    s.sdo_write(0x1C12, 0, struct.pack("B", 0))
    s.sdo_write(0x1600, 0, struct.pack("B", 0))
    s.sdo_write(0x1600, 1, struct.pack("<I", 0x60400010))
    s.sdo_write(0x1600, 2, struct.pack("<I", 0x607A0020))
    s.sdo_write(0x1600, 3, struct.pack("<I", 0x60810020))
    s.sdo_write(0x1600, 0, struct.pack("B", 3))
    s.sdo_write(0x1C12, 1, struct.pack("<H", 0x1600))
    s.sdo_write(0x1C12, 0, struct.pack("B", 1))
    s.sdo_write(0x1C13, 0, struct.pack("B", 0))
    s.sdo_write(0x1A00, 0, struct.pack("B", 0))
    s.sdo_write(0x1A00, 1, struct.pack("<I", 0x60410010))
    s.sdo_write(0x1A00, 2, struct.pack("<I", 0x60640020))
    s.sdo_write(0x1A00, 3, struct.pack("<I", 0x606C0020))
    s.sdo_write(0x1A00, 0, struct.pack("B", 3))
    s.sdo_write(0x1C13, 1, struct.pack("<H", 0x1A00))
    s.sdo_write(0x1C13, 0, struct.pack("B", 1))
    s.sdo_write(0x6060, 0, struct.pack("b", 1))
    s.sdo_write(0x6083, 0, struct.pack("<I", ACCEL))
    s.sdo_write(0x6084, 0, struct.pack("<I", ACCEL))
    s.sdo_write(0x6081, 0, struct.pack("<I", SPEED))


def main():
    global sl
    print(f"[i] iface={IFACE}")
    master.open(IFACE)
    n = master.config_init()
    print(f"[i] 슬레이브 {n}개")
    if n <= 0:
        return
    sl = master.slaves[0]
    sl.config_func = cfg
    master.config_map()
    print(f"[i] SAFEOP={hex(master.state_check(pysoem.SAFEOP_STATE, 50000))}")
    sl.output = struct.pack("<HiI", 0, 0, SPEED)
    master.send_processdata(); master.receive_processdata(2000)
    master.state = pysoem.OP_STATE
    master.write_state()
    for _ in range(300):
        master.send_processdata(); master.receive_processdata(2000)
        if master.state_check(pysoem.OP_STATE, 5000) == pysoem.OP_STATE:
            break
        time.sleep(0.002)
    print(f"[i] OP={hex(master.state_check(pysoem.OP_STATE, 5000))}")

    print("\n===== 정적 진단 (SDO) =====")
    print(f"  6060h설정모드={u(0x6060,0,'b')}  6061h현재모드={u(0x6061,0,'b')}  (1=PP)")
    print(f"  603Fh에러=0x{u(0x603F):04X}")
    print(f"  2100h 입력HW(I1~I5 bit0~4)={dec_bits(u(0x2100))}")
    print(f"  60FDh 기능상태={dec_fd(u(0x60FD,0,'<i'))}   ← NegLim/PosLim/Home 여기 뜨면 리미트 활성")
    print(f"  2500h 입력기본레벨(0=NO,1=NC 비트별)={u(0x2500):#06x}")
    print(f"  DI기능: I1(2510)={u(0x2510)} I2(2511)={u(0x2511)} I3(2512)={u(0x2512)} "
          f"I4(2513)={u(0x2513)} I5(2514)={u(0x2514)}")
    print("  (기능값 0=미정 1=Home 2=+Limit 3=-Limit 4=Stop 5=E-Stop 6=Enable 7=Probe1 8=Probe2)")
    print(f"  2403h 브레이크Enable={u(0x2403)}")

    print("\n===== OP 진입 직후 상태 샘플 (cw=0) =====")
    for i in range(5):
        sw, apos, _, wkc = io(0x00, 0)
        print(f"  s{i}: {dec(sw)} pos={apos} 60FD={dec_fd(u(0x60FD,0,'<i'))}")
        time.sleep(0.02)

    print("\n===== Enable 시퀀스 (각 단계 길게 hold + 샘플) =====")
    for label, cw, hold in [("shutdown0x06", 0x06, 30), ("switchon0x07", 0x07, 30)]:
        sw, apos, _, wkc = pump(cw, 0, hold)
        print(f"  [{label}] {dec(sw)}")

    print("  [enable0x0F] 100사이클 hold하며 10마다 샘플:")
    for k in range(10):
        sw, apos, aspd, wkc = pump(0x0F, 0, 10)
        print(f"     k={k}: {dec(sw)} pos={apos} 60FD={dec_fd(u(0x60FD,0,'<i'))} err=0x{u(0x603F):04X}")

    enabled = bool(sw & 0x0004)
    print(f"\n[결과] Operation Enabled = {'✅ 성공' if enabled else '❌ 실패'}  최종 {dec(sw)}")

    if enabled and DO_MOVE:
        print("\n===== 5° 상대이동 =====")
        base = 0x0F | 0x40
        pump(base, STEP, 3)
        pump(base | 0x10, STEP, 3)
        start = io(base, STEP)[1]
        for i in range(60):
            sw, apos, aspd, wkc = io(base, STEP)
            if i % 10 == 0:
                print(f"   t={i*0.02:.2f}s {dec(sw)} pos={apos:+d} spd={aspd:+d}")
            time.sleep(0.02)
        print(f"   이동량={apos-start:+d} pulse (목표 {STEP})")

    cleanup()


def dec_bits(v):
    if isinstance(v, str):
        return v
    return "".join(str((v >> i) & 1) for i in range(5)) + f" (I1..I5) raw=0x{v:04X}"


def cleanup():
    try:
        pump(0x00, 0, 5)
        master.state = pysoem.INIT_STATE
        master.write_state()
    except Exception:
        pass
    master.close()
    print("[i] 종료")


if __name__ == "__main__":
    main()
