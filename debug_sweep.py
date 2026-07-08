#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""컨트롤워드 실측 스윕: 어떤 값이 Operation Enabled(bit2)를 켜는지 찾는다.
모터는 안 움직임(enable만 테스트)."""
import sys, struct, time
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
SPEED = 5000
master = pysoem.Master()
sl = None


def dec(sw):
    b = [("RDY", 1), ("ON", 2), ("ENA", 4), ("FLT", 8), ("VOLT", 0x10),
         ("QS", 0x20), ("NRDY", 0x40), ("REM", 0x200), ("TRGT", 0x400)]
    return f"0x{sw:04X}[" + " ".join(n for n, m in b if sw & m) + "]"


def io(cw):
    sl.output = struct.pack("<HiI", cw & 0xFFFF, 0, SPEED)
    master.send_processdata()
    master.receive_processdata(2000)
    return struct.unpack("<Hii", bytes(sl.input)[:10])[0]


def pump(cw, n):
    sw = 0
    for _ in range(n):
        sw = io(cw)
        time.sleep(0.004)
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
    print(f"OP={hex(master.state_check(pysoem.OP_STATE,5000))}\n")

    # 후보 최종 컨트롤워드 (각각: reset→0x06→0x07→후보)
    candidates = [0x0F, 0x0B, 0x0D, 0x09, 0x07, 0x1F, 0x2F, 0x3F, 0x8F]
    print("각 후보: reset→0x06→0x07→[후보] 순으로 넣고 결과 status:")
    for c in candidates:
        pump(0x80, 8)   # fault reset
        pump(0x00, 8)
        pump(0x06, 12)
        s7 = pump(0x07, 12)
        sc = pump(c, 25)
        ena = "✅ENABLED" if sc & 0x0004 else ""
        print(f"  cw=0x{c:02X} → {dec(sc)}  (0x07직후={dec(s7)}) {ena}")

    # 추가: 브레이크 자동제어 끄고(2403=0) 표준 enable 재시도
    print("\n[추가] 2403h=0 (브레이크 자동제어 OFF) 후 표준 enable:")
    sl.sdo_write(0x2403, 0, struct.pack("<H", 0))
    pump(0x80, 8); pump(0x00, 8); pump(0x06, 12); pump(0x07, 12)
    sc = pump(0x0F, 25)
    print(f"  0x0F → {dec(sc)} {'✅ENABLED' if sc & 4 else ''}")
    sl.sdo_write(0x2403, 0, struct.pack("<H", 1))  # 원복

    pump(0x00, 5)
    master.state = pysoem.INIT_STATE; master.write_state()
    master.close()
    print("\n종료")


if __name__ == "__main__":
    main()
