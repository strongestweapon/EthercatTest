#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""브레이크 측정용: 드라이버를 지정 상태로 20초 유지.
사용:  python3 brake_hold.py disable   (또는 enable)"""
import sys, struct, time
import pysoem

IFACE = "en7"
MODE = sys.argv[1] if len(sys.argv) > 1 else "disable"
SECS = int(sys.argv[2]) if len(sys.argv) > 2 else 20
master = pysoem.Master()
sl = None


def io(cw):
    sl.output = struct.pack("<HiI", cw & 0xFFFF, 0, 5000)
    master.send_processdata(); master.receive_processdata(2000)
    return struct.unpack("<Hii", bytes(sl.input)[:10])[0]


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


def main():
    global sl
    master.open(IFACE)
    if master.config_init() <= 0:
        print("슬레이브 없음 (앱 닫았는지 확인)"); return
    sl = master.slaves[0]
    sl.config_func = cfg
    master.config_map()
    master.state_check(pysoem.SAFEOP_STATE, 50000)
    sl.output = struct.pack("<HiI", 0, 0, 5000)
    master.send_processdata(); master.receive_processdata(2000)
    master.state = pysoem.OP_STATE; master.write_state()
    for _ in range(300):
        master.send_processdata(); master.receive_processdata(2000)
        if master.state_check(pysoem.OP_STATE, 5000) == pysoem.OP_STATE:
            break
        time.sleep(0.002)
    sl.sdo_write(0x2403, 0, struct.pack("<H", 1))     # 브레이크 자동제어 ON
    print("2403h =", struct.unpack("<H", sl.sdo_read(0x2403, 0)[:2])[0])

    if MODE == "enable":
        for cw in (0x80, 0x00, 0x06, 0x07):
            for _ in range(200):
                io(cw); time.sleep(0.004)
        ok = False
        for _ in range(1000):
            if io(0x0F) & 4:
                ok = True; break
            time.sleep(0.004)
        print("ENABLE", "OK" if ok else "실패", f"status=0x{io(0x0F):04X}")
        hold_cw = 0x0F
        print(f"\n★★ ENABLE 상태 {SECS}초 유지 — 지금 BK+/BK-(코일) 전압 측정! (24V 뜨는지) ★★")
    else:
        hold_cw = 0x0000
        print(f"\n=== DISABLE 상태 {SECS}초 유지 — 지금 BK+/BK- 측정 ===")

    for t in range(SECS, 0, -1):
        print(f"   {t}초  (status=0x{io(hold_cw):04X})", flush=True)
        for _ in range(250):
            io(hold_cw); time.sleep(0.004)

    io(0x0000)
    master.state = pysoem.INIT_STATE; master.write_state()
    master.close()
    print("종료")


if __name__ == "__main__":
    main()
