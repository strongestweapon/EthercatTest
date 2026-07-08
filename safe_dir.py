#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""폭주 방지 안전 이동 테스트. 2300h(방향) 현재값/반전값 각각 소량 이동 시도.
150펄스(~5°) 이상 벗어나면 즉시 정지."""
import sys, struct, time
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
SPEED = 2000          # 느리게 (폭주 대비)
LIMIT = 150           # 안전 한계 펄스 (~5°)
master = pysoem.Master()
sl = None


def io(cw, target):
    sl.output = struct.pack("<HiI", cw & 0xFFFF, target, SPEED)
    master.send_processdata(); master.receive_processdata(2000)
    return struct.unpack("<Hii", bytes(sl.input)[:10])


def pump(cw, target, n):
    r = (0, 0, 0)
    for _ in range(n):
        r = io(cw, target); time.sleep(0.004)
    return r


def si(idx, fmt="<i"):
    return struct.unpack(fmt, sl.sdo_read(idx, 0)[:struct.calcsize(fmt)])[0]


def cfg(pos):
    s = master.slaves[pos]
    for idx, sub, val, f in [
        (0x1C12, 0, 0, "B"), (0x1600, 0, 0, "B"),
        (0x1600, 1, 0x60400010, "<I"), (0x1600, 2, 0x607A0020, "<I"), (0x1600, 3, 0x60810020, "<I"),
        (0x1600, 0, 3, "B"), (0x1C12, 1, 0x1600, "<H"), (0x1C12, 0, 1, "B"),
        (0x1C13, 0, 0, "B"), (0x1A00, 0, 0, "B"),
        (0x1A00, 1, 0x60410010, "<I"), (0x1A00, 2, 0x60640020, "<I"), (0x1A00, 3, 0x606C0020, "<I"),
        (0x1A00, 0, 3, "B"), (0x1C13, 1, 0x1A00, "<H"), (0x1C13, 0, 1, "B"),
        (0x6060, 0, 1, "b"), (0x6083, 0, 20000, "<I"), (0x6084, 0, 20000, "<I"), (0x6081, 0, SPEED, "<I"),
    ]:
        s.sdo_write(idx, sub, struct.pack(f, val))


def enable():
    pump(0x80, 0, 15); pump(0x00, 0, 15)
    pump(0x06, 0, 15); pump(0x07, 0, 15)
    for _ in range(1000):
        if io(0x0F, 0)[0] & 0x0004:
            return True
        time.sleep(0.004)
    return False


def safe_move(cmd_pulse):
    """cmd_pulse만큼 상대이동 시도. LIMIT 넘으면 즉시 정지."""
    start = io(0x0F, 0)[1]
    base = 0x0F | 0x40                       # relative
    pump(base, cmd_pulse, 3)
    pump(base | 0x10, cmd_pulse, 3)          # 트리거
    verdict = "정지(움직임없음)"
    last = start
    for i in range(200):                     # 최대 ~1s
        sw, apos, aspd = io(base, cmd_pulse)
        last = apos
        d = apos - start
        if abs(d) > LIMIT:
            # 폭주! 즉시 정지
            for _ in range(5):
                io(0x00, 0)
            verdict = f"⚠️폭주감지 {d:+d}펄스 → 즉시정지"
            break
        if sw & 0x0008:
            verdict = f"폴트 (이동 {d:+d})"
            break
        if (sw & 0x0400) and abs(apos - (start + cmd_pulse)) < 20:
            verdict = f"✅목표도달 (이동 {d:+d}/{cmd_pulse:+d})"
            break
        time.sleep(0.005)
    d = last - start
    return d, verdict


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

    dir0 = struct.unpack("<H", sl.sdo_read(0x2300, 0)[:2])[0]
    print(f"현재 2300h(방향)={dir0}\n")

    for label, dval in [(f"방향={dir0}(현재)", dir0), (f"방향={1 - dir0}(반전)", 1 - dir0)]:
        # 방향 설정 (라이브 시도)
        sl.sdo_write(0x2300, 0, struct.pack("<H", dval))
        rb = struct.unpack("<H", sl.sdo_read(0x2300, 0)[:2])[0]
        pump(0x00, 0, 10)
        if not enable():
            print(f"[{label}] enable 실패"); continue
        d, v = safe_move(100)     # +1도 살짝
        print(f"[{label}] 되읽기={rb}  → +100펄스 명령: {v}")
        pump(0x00, 0, 10)         # disable 후 다음

    # 원복
    sl.sdo_write(0x2300, 0, struct.pack("<H", dir0))
    pump(0x00, 0, 5)
    master.state = pysoem.INIT_STATE; master.write_state()
    master.close()
    print("\n종료 (2300h 원복, disable)")


if __name__ == "__main__":
    main()
