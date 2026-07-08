#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""긴 이동(+3000)으로 속도/가속 조합 스윕. 반대방향 폭주 즉시정지."""
import sys, struct, time
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
master = pysoem.Master()
sl = None
_spd = 5000


def io(cw, target):
    sl.output = struct.pack("<HiI", cw & 0xFFFF, target, _spd)
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
        (0x6060, 0, 1, "b"),
    ]:
        s.sdo_write(idx, sub, struct.pack(f, val))


def enable():
    pump(0x80, 0, 12); pump(0x00, 0, 12); pump(0x06, 0, 12); pump(0x07, 0, 12)
    for _ in range(1000):
        if io(0x0F, 0)[0] & 0x0004:
            return True
        time.sleep(0.004)
    return False


def trymove(speed, accel, cmd=3000):
    global _spd
    _spd = speed
    sl.sdo_write(0x6083, 0, struct.pack("<I", accel))
    sl.sdo_write(0x6084, 0, struct.pack("<I", accel))
    if not enable():
        return "enable실패"
    start = io(0x0F, 0)[1]
    base = 0x0F | 0x40
    pump(base, cmd, 3)
    pump(base | 0x10, cmd, 3)
    peakspd = 0
    for i in range(400):
        sw, apos, aspd = io(base, cmd)
        d = apos - start
        peakspd = max(peakspd, abs(aspd))
        if d < -100:
            for _ in range(6):
                io(0x00, 0)
            return f"⚠️폭주(반대 {d:+d})"
        if sw & 0x0008:
            return f"폴트({d:+d})"
        if abs(apos - (start + cmd)) < 30:
            pump(0x00, 0, 8)
            return f"✅도달({d:+d}/{cmd} 피크속도{peakspd})"
        time.sleep(0.004)
    r = f"미완({io(0x0F,0)[1]-start:+d})"
    pump(0x00, 0, 8)
    return r


def main():
    global sl
    master.open(IFACE)
    if master.config_init() <= 0:
        print("슬레이브 없음"); return
    sl = master.slaves[0]
    sl.config_func = cfg
    master.config_map()
    master.state_check(pysoem.SAFEOP_STATE, 50000)
    sl.output = struct.pack("<HiI", 0, 0, _spd)
    master.send_processdata(); master.receive_processdata(2000)
    master.state = pysoem.OP_STATE; master.write_state()
    for _ in range(300):
        master.send_processdata(); master.receive_processdata(2000)
        if master.state_check(pysoem.OP_STATE, 5000) == pysoem.OP_STATE:
            break
        time.sleep(0.002)

    print("(속도, 가속) +3000펄스:")
    for spd, acc in [(5000, 50000), (10000, 50000), (10000, 100000),
                     (20000, 50000), (20000, 100000), (20000, 200000)]:
        print(f"  spd={spd:6d} acc={acc:7d}: {trymove(spd, acc)}")
        pump(0x00, 0, 5)

    pump(0x00, 0, 5)
    master.state = pysoem.INIT_STATE; master.write_state()
    master.close()
    print("종료")


if __name__ == "__main__":
    main()
