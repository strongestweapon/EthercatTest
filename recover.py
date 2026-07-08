#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""폴트 코드 확인 → Fault Reset → enable → 90° 이동 실측."""
import sys, struct, time
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
SPEED = 5000
master = pysoem.Master()
sl = None

ERR = {0x0000: "정상", 0xFF01: "과전류", 0xFF02: "과전압", 0xFF03: "저전압",
       0xFF04: "상오류", 0xFF05: "위치편차(following error)"}


def dec(sw):
    b = [("RDY", 1), ("ON", 2), ("ENA", 4), ("FLT", 8), ("VOLT", 0x10),
         ("QS", 0x20), ("REM", 0x200), ("TRGT", 0x400), ("ACK", 0x1000)]
    return f"0x{sw:04X}[" + " ".join(n for n, m in b if sw & m) + "]"


def io(cw, target=0):
    sl.output = struct.pack("<HiI", cw & 0xFFFF, target, SPEED)
    master.send_processdata(); master.receive_processdata(2000)
    return struct.unpack("<Hii", bytes(sl.input)[:10])


def pump(cw, target, n):
    r = (0, 0, 0)
    for _ in range(n):
        r = io(cw, target); time.sleep(0.004)
    return r


def err():
    return struct.unpack("<H", sl.sdo_read(0x603F, 0)[:2])[0]


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

    sw, apos, _ = io(0x00)
    e = err()
    print(f"[초기] {dec(sw)} pos={apos} 에러=0x{e:04X}({ERR.get(e,'?')})")

    print("[리셋] 0x80 → 0x00")
    pump(0x80, 0, 15); pump(0x00, 0, 15)
    sw, apos, _ = io(0x00)
    e = err()
    print(f"[리셋후] {dec(sw)} 에러=0x{e:04X}({ERR.get(e,'?')})")

    print("[enable] 0x06→0x07→0x0F 폴링...")
    pump(0x06, 0, 15); pump(0x07, 0, 15)
    ok = False
    for i in range(1000):
        sw = io(0x0F)[0]
        if sw & 0x0004:
            ok = True
            print(f"  ✅ enable {(i*4)}ms {dec(sw)}"); break
        if sw & 0x0008:
            print(f"  ❌ enable중 FLT {dec(sw)} 에러=0x{err():04X}({ERR.get(err(),'?')})"); break
        time.sleep(0.004)
    if not ok:
        print(f"  최종 {dec(io(0x0F)[0])}"); cleanup(); return

    # 90° 이동
    step = int(round(90 * 10000 / 360))
    print(f"[이동] 90° = {step} pulse")
    base = 0x0F | 0x40
    start = io(base, step)[1]
    pump(base, step, 5)
    r = pump(base | 0x10, step, 5)
    print(f"  트리거 {dec(r[0])}")
    end = start
    for i in range(150):
        sw, end, spd = io(base, step)
        if i % 15 == 0:
            print(f"   t={i*0.02:4.2f}s {dec(sw)} pos={end:+d} spd={spd:+d} err=0x{err():04X}")
        time.sleep(0.02)
    print(f"[결과] {start} → {end} 이동={end-start:+d}/{step} 에러=0x{err():04X}({ERR.get(err(),'?')})")
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
