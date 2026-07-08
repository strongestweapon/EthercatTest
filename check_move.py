#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""이동 중 607A(목표)/6081(속도)/6064(실제)/606C(속도) SDO 실측으로 원인 규명."""
import sys, struct, time
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
SPEED = 20000          # 넉넉히
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


def si(idx, sub=0, fmt="<i"):
    return struct.unpack(fmt, sl.sdo_read(idx, sub)[:struct.calcsize(fmt)])[0]


def cfg(pos):
    s = master.slaves[pos]
    for idx, sub, val, f in [
        (0x1C12, 0, 0, "B"), (0x1600, 0, 0, "B"),
        (0x1600, 1, 0x60400010, "<I"), (0x1600, 2, 0x607A0020, "<I"), (0x1600, 3, 0x60810020, "<I"),
        (0x1600, 0, 3, "B"), (0x1C12, 1, 0x1600, "<H"), (0x1C12, 0, 1, "B"),
        (0x1C13, 0, 0, "B"), (0x1A00, 0, 0, "B"),
        (0x1A00, 1, 0x60410010, "<I"), (0x1A00, 2, 0x60640020, "<I"), (0x1A00, 3, 0x606C0020, "<I"),
        (0x1A00, 0, 3, "B"), (0x1C13, 1, 0x1A00, "<H"), (0x1C13, 0, 1, "B"),
        (0x6060, 0, 1, "b"), (0x6083, 0, 100000, "<I"), (0x6084, 0, 100000, "<I"), (0x6081, 0, SPEED, "<I"),
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

    pump(0x80, 0, 15); pump(0x00, 0, 15)       # reset
    pump(0x06, 0, 15); pump(0x07, 0, 15)
    for _ in range(1000):
        if io(0x0F, 0)[0] & 0x0004:
            break
        time.sleep(0.004)
    cur = si(0x6064)
    print(f"enable OK, 현재 6064={cur}")
    print(f"  6081(속도)={si(0x6081,0,'<I')}  6083(가속)={si(0x6083,0,'<I')}  6060/6061={sl.sdo_read(0x6060,0)[0]}/{sl.sdo_read(0x6061,0)[0]}")

    # 절대이동: target = cur + 2500
    tgt = cur + 2500
    print(f"\n절대이동 target 607A={tgt} (현재+2500), 트리거 0x0F→0x1F")
    pump(0x0F, tgt, 5)
    r = pump(0x1F, tgt, 3)
    print(f"  트리거직후 {dec(r[0])}")
    for i in range(20):
        sw, apos, aspd = io(0x0F, tgt)
        s607a = si(0x607A); s6081 = si(0x6081, 0, "<I"); s606c = si(0x606C)
        print(f"  t={i*0.1:.1f}s {dec(sw)} 6064={apos} 607A={s607a} 6081={s6081} 606C={s606c}")
        time.sleep(0.1)
    print(f"[결과] 최종 6064={si(0x6064)} (목표 {tgt}, 이동 {si(0x6064)-cur:+d}/2500)")

    pump(0x00, 0, 5)
    master.state = pysoem.INIT_STATE; master.write_state()
    master.close()
    print("종료")


if __name__ == "__main__":
    main()
