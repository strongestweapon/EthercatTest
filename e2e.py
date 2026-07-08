#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""end-to-end: enable(폴링) → 5° 상대이동 → 위치추적. 왜 안 움직이는지 전부 출력."""
import sys, struct, time
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
SPEED = 5000
STEP = int(round(5.0 / 360.0 * 10000))   # 139 pulse
master = pysoem.Master()
sl = None


def dec(sw):
    b = [("RDY", 1), ("ON", 2), ("ENA", 4), ("FLT", 8), ("VOLT", 0x10),
         ("QS", 0x20), ("REM", 0x200), ("TRGT", 0x400), ("ACK", 0x1000)]
    return f"0x{sw:04X}[" + " ".join(n for n, m in b if sw & m) + "]"


def io(cw, target):
    sl.output = struct.pack("<HiI", cw & 0xFFFF, target, SPEED)
    master.send_processdata(); master.receive_processdata(2000)
    return struct.unpack("<Hii", bytes(sl.input)[:10])


def pump(cw, target, n):
    r = (0, 0, 0)
    for _ in range(n):
        r = io(cw, target); time.sleep(0.004)
    return r


def cfg(pos):
    s = master.slaves[pos]
    for idx, sub, val, f in [
        (0x1C12, 0, 0, "B"), (0x1600, 0, 0, "B"),
        (0x1600, 1, 0x60400010, "<I"), (0x1600, 2, 0x607A0020, "<I"), (0x1600, 3, 0x60810020, "<I"),
        (0x1600, 0, 3, "B"), (0x1C12, 1, 0x1600, "<H"), (0x1C12, 0, 1, "B"),
        (0x1C13, 0, 0, "B"), (0x1A00, 0, 0, "B"),
        (0x1A00, 1, 0x60410010, "<I"), (0x1A00, 2, 0x60640020, "<I"), (0x1A00, 3, 0x606C0020, "<I"),
        (0x1A00, 0, 3, "B"), (0x1C13, 1, 0x1A00, "<H"), (0x1C13, 0, 1, "B"),
        (0x6060, 0, 1, "b"), (0x6083, 0, 50000, "<I"), (0x6084, 0, 50000, "<I"), (0x6081, 0, SPEED, "<I"),
    ]:
        s.sdo_write(idx, sub, struct.pack(f, val))


def main():
    global sl
    master.open(IFACE)
    if master.config_init() <= 0:
        print("슬레이브 없음"); return
    sl = master.slaves[0]
    sl.config_func = cfg
    master.config_map()
    master.state_check(pysoem.SAFEOP_STATE, 50000)
    sl.output = struct.pack("<HiI", 0, 0, SPEED)
    master.send_processdata(); master.receive_processdata(2000)
    master.state = pysoem.OP_STATE; master.write_state()
    for _ in range(300):
        master.send_processdata(); master.receive_processdata(2000)
        if master.state_check(pysoem.OP_STATE, 5000) == pysoem.OP_STATE:
            break
        time.sleep(0.002)
    print(f"OP={hex(master.state_check(pysoem.OP_STATE,5000))}  6061모드={sl.sdo_read(0x6061,0)[0]}")

    # ---- Enable (폴링) ----
    pump(0x06, 0, 15); pump(0x07, 0, 15)
    t0 = time.perf_counter()
    for _ in range(1000):
        sw, apos, aspd = io(0x0F, 0)
        if sw & 0x0004:
            print(f"[enable] ✅ {dec(sw)} {(time.perf_counter()-t0)*1000:.0f}ms  pos={apos}")
            break
        time.sleep(0.004)
    else:
        print("[enable] ❌ 실패"); cleanup(); return

    # ---- 상대이동 5° ----
    print(f"\n[move] 상대 {STEP} pulse (5°), speed={SPEED}")
    base = 0x0F | 0x40          # enabled + relative
    r = pump(base, STEP, 5)
    print(f"  base준비 {dec(r[0])} pos={r[1]}")
    # bit4 0->1 상승엣지
    r = pump(base | 0x10, STEP, 5)   # 0x5F
    print(f"  트리거!  {dec(r[0])} pos={r[1]}  (ACK=setpoint 수신)")
    start = r[1]
    # 관찰 2초 (bit4 내림)
    for i in range(100):
        sw, apos, aspd = io(base, STEP)
        if i % 5 == 0:
            print(f"   t={i*0.02:4.2f}s {dec(sw)} pos={apos:+6d} spd={aspd:+6d}")
        time.sleep(0.02)
    print(f"[결과] 이동량={apos-start:+d} pulse (목표 {STEP}) → {'✅움직임' if abs(apos-start)>5 else '❌안움직임'}")
    print(f"       최종에러=0x{struct.unpack('<H', sl.sdo_read(0x603F,0)[:2])[0]:04X}")

    # ---- 안 움직이면 절대이동도 시도 ----
    if abs(apos - start) <= 5:
        print("\n[move2] 절대이동 시도 (target=현재+139)")
        tgt = apos + STEP
        pump(0x0F, tgt, 5)
        r = pump(0x1F, tgt, 5)     # abs, bit4 rising
        print(f"  트리거! {dec(r[0])} pos={r[1]}")
        s2 = r[1]
        for i in range(100):
            sw, apos, aspd = io(0x0F, tgt)
            if i % 5 == 0:
                print(f"   t={i*0.02:4.2f}s {dec(sw)} pos={apos:+6d} spd={aspd:+6d}")
            time.sleep(0.02)
        print(f"[결과2] 이동량={apos-s2:+d} → {'✅' if abs(apos-s2)>5 else '❌'}")

    cleanup()


def cleanup():
    try:
        pump(0x00, 0, 5)
        master.state = pysoem.INIT_STATE; master.write_state()
    except Exception:
        pass
    master.close()
    print("종료")


if __name__ == "__main__":
    main()
