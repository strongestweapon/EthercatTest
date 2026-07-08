#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GUI와 똑같은 이동 로직(3사이클)으로 5°반복·90°를 명령하고 엔코더 실측."""
import sys, struct, time
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
SPEED = 5000
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


def gui_move(step_pulse, hold_ms=1500):
    """GUI 방식 그대로: base(0x4F) 3cyc → trig(0x5F) 3cyc → base 유지."""
    base = 0x0F | 0x40
    start = io(base, step_pulse)[1]
    pump(base, step_pulse, 3)
    r = pump(base | 0x10, step_pulse, 3)     # 트리거
    ack = "ACK" if r[0] & 0x1000 else "no-ack"
    # 유지하며 관찰
    end = start
    for _ in range(int(hold_ms / 20)):
        end = io(base, step_pulse)[1]
        time.sleep(0.02)
    return start, end, ack


def main():
    global sl
    master.open(IFACE)
    if master.config_init() <= 0:
        print("슬레이브 없음(GUI 닫았는지 확인)"); return
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

    # enable (폴링)
    pump(0x06, 0, 15); pump(0x07, 0, 15)
    for _ in range(1000):
        sw = io(0x0F, 0)[0]
        if sw & 0x0004:
            break
        time.sleep(0.004)
    print(f"enable={dec(sw)}\n")

    P = 10000 / 360.0
    print("=== GUI 방식 이동 실측 (엔코더 delta) ===")
    for deg in [5, 5, 5, 30, 90]:
        step = int(round(deg * P))
        s0, s1, ack = gui_move(step)
        moved = s1 - s0
        print(f"  {deg:3d}° (목표{step:+5d}) → 엔코더 {s0} → {s1}  이동 {moved:+5d}  "
              f"[{ack}] {'✅' if abs(moved) > step * 0.5 else '❌안움직임'}")

    pump(0x00, 0, 5)
    master.state = pysoem.INIT_STATE; master.write_state()
    master.close()
    print("종료")


if __name__ == "__main__":
    main()
