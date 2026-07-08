#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""정상동작 데모: 여러 바퀴 정/역회전."""
import sys, struct, time
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
master = pysoem.Master()
sl = None
_spd = 12000
PPR = 10000


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
        (0x6060, 0, 1, "b"), (0x6083, 0, 100000, "<I"), (0x6084, 0, 100000, "<I"),
    ]:
        s.sdo_write(idx, sub, struct.pack(f, val))


def enable():
    pump(0x80, 0, 12); pump(0x00, 0, 12); pump(0x06, 0, 12); pump(0x07, 0, 12)
    for _ in range(1000):
        if io(0x0F, 0)[0] & 0x0004:
            return True
        time.sleep(0.004)
    return False


def move_rev(revs):
    cmd = int(revs * PPR)
    start = io(0x0F, 0)[1]
    base = 0x0F | 0x40
    pump(base, cmd, 3)
    pump(base | 0x10, cmd, 3)
    for _ in range(2500):
        sw, apos, aspd = io(base, cmd)
        if sw & 0x0008:
            return f"폴트 ({apos-start:+d})"
        if abs(apos - (start + cmd)) < 30:
            return f"✅ {revs:+.0f}바퀴 완료 (이동 {apos-start:+d}/{cmd})"
        time.sleep(0.004)
    return f"미완 ({io(0x0F,0)[1]-start:+d}/{cmd})"


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
    if not enable():
        print("enable 실패"); return
    print(f"enable OK, 속도={_spd} pulse/s (~{_spd/PPR:.1f} rev/s)")
    print("촬영 준비... 5초 후 시작", flush=True)
    for c in range(5, 0, -1):
        print(f"  {c}...", flush=True)
        pump(0x0F, 0, 250)     # 제자리 홀딩 ~1초
    print("시작!\n", flush=True)

    for revs in [2, -2, 3, -3, 5, -5]:
        print(f"  {move_rev(revs)}", flush=True)
        pump(0x0F, 0, 250)     # 1초 정지 (촬영용)

    pump(0x00, 0, 8)
    master.state = pysoem.INIT_STATE; master.write_state()
    master.close()
    print("\n종료 (disable)")


if __name__ == "__main__":
    main()
