#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""브레이크 BK 출력 검증: 2403h=1 arm → enable → 8초 유지(그동안 BK전압 측정).
드라이버 내부 출력상태(2101h/60FE)도 함께 관찰."""
import sys, struct, time
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
SPEED = 5000
master = pysoem.Master()
sl = None


def io(cw, t=0):
    sl.output = struct.pack("<HiI", cw & 0xFFFF, t, SPEED)
    master.send_processdata(); master.receive_processdata(2000)
    return struct.unpack("<Hii", bytes(sl.input)[:10])


def pump(cw, t, n):
    r = (0, 0, 0)
    for _ in range(n):
        r = io(cw, t); time.sleep(0.004)
    return r


def u(idx, sub=0):
    try:
        return struct.unpack("<H", sl.sdo_read(idx, sub)[:2])[0]
    except Exception as e:
        return f"err:{e}"


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
    sl.output = struct.pack("<HiI", 0, 0, SPEED)
    master.send_processdata(); master.receive_processdata(2000)
    master.state = pysoem.OP_STATE; master.write_state()
    for _ in range(300):
        master.send_processdata(); master.receive_processdata(2000)
        if master.state_check(pysoem.OP_STATE, 5000) == pysoem.OP_STATE:
            break
        time.sleep(0.002)

    print("=== 브레이크 파라미터 (설정 전) ===")
    print(f"  2403h BrakeEnable = {u(0x2403)}  (1이어야 자동제어 ON)")
    print(f"  2404h OpenDelay   = {u(0x2404)} ms")
    print(f"  2405h CloseDelay  = {u(0x2405)} ms")
    print(f"  2101h 출력상태(설정전) = 0x{u(0x2101):04X}")

    print("\n[1] 2403h=1 (브레이크 자동제어 ON) 세팅")
    sl.sdo_write(0x2403, 0, struct.pack("<H", 1))
    print(f"    되읽기 2403h = {u(0x2403)}")

    print("[2] Enable (0x06→0x07→0x0F, ~1초)")
    pump(0x80, 0, 12); pump(0x00, 0, 12); pump(0x06, 0, 15); pump(0x07, 0, 15)
    ok = False
    for _ in range(1000):
        if io(0x0F)[0] & 0x0004:
            ok = True; break
        time.sleep(0.004)
    print(f"    enable = {'OK' if ok else '실패'}  status=0x{io(0x0F)[0]:04X}")

    print("\n[3] ★★ 지금 BK+/BK- 전압 측정하세요! 10초간 enable 유지 ★★")
    for t in range(10, 0, -1):
        o = u(0x2101)
        print(f"    유지중... {t}초  (2101h출력={('0x%04X'%o) if isinstance(o,int) else o})")
        pump(0x0F, 0, 250)   # ~1초 유지

    print("\n[4] Disable")
    pump(0x00, 0, 10)
    print(f"    2101h 출력상태(disable후) = 0x{u(0x2101):04X}")

    master.state = pysoem.INIT_STATE; master.write_state()
    master.close()
    print("종료")


if __name__ == "__main__":
    main()
