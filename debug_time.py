#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""0x0F 유지 시 언제 ENA(bit2)가 켜지는지 시간 측정."""
import sys, struct, time
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
SPEED = 5000
master = pysoem.Master()
sl = None


def io(cw):
    sl.output = struct.pack("<HiI", cw & 0xFFFF, 0, SPEED)
    master.send_processdata(); master.receive_processdata(2000)
    return struct.unpack("<Hii", bytes(sl.input)[:10])[0]


def pump(cw, n):
    sw = 0
    for _ in range(n):
        sw = io(cw); time.sleep(0.004)
    return sw


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

    pump(0x06, 15); pump(0x07, 15)
    print("0x0F 유지, ENA 켜질 때까지 측정:")
    t0 = time.perf_counter()
    ena_t = None
    for i in range(1000):        # 최대 4초
        sw = io(0x0F)
        if (sw & 0x0004) and ena_t is None:
            ena_t = time.perf_counter() - t0
            print(f"  ✅ ENA ON at {ena_t*1000:.0f} ms  status=0x{sw:04X}")
            break
        if i % 25 == 0:
            print(f"  t={ (time.perf_counter()-t0)*1000:5.0f}ms status=0x{sw:04X}")
        time.sleep(0.004)
    if ena_t is None:
        print("  ❌ 4초 내 ENA 안 켜짐")

    pump(0x00, 5)
    master.state = pysoem.INIT_STATE; master.write_state()
    master.close()
    print("종료")


if __name__ == "__main__":
    main()
