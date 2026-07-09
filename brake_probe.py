#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""검증: enable 시퀀스의 클리어(0x00)/폴트리셋(0x80)이 브레이크 설정(2403h)을 날리나?
각 컨트롤워드 단계 직후 2403h를 SDO로 읽어 값 변화를 로그. 추측 배제, 실측.
사용:  sudo python3 brake_probe.py [en7]
"""
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
    r = 0
    for _ in range(n):
        r = io(cw); time.sleep(0.004)
    return r


def rd2403():
    try:
        return struct.unpack("<H", sl.sdo_read(0x2403, 0)[:2])[0]
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
        print("슬레이브 없음 (GUI 앱 닫았는지 확인)"); return
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

    print(f"OP 진입 후    2403h = {rd2403()}")
    sl.sdo_write(0x2403, 0, struct.pack("<H", 1))
    time.sleep(0.05)
    print(f"2403h=1 쓰기 후 2403h = {rd2403()}   <- 여기서 1이어야 시작")

    # enable 시퀀스를 한 단계씩, 각 직후 2403h 읽기
    steps = [(0x0080, "0x80 폴트리셋"), (0x0000, "0x00 클리어"),
             (0x0006, "0x06"), (0x0007, "0x07"), (0x000F, "0x0F enable")]
    for cw, name in steps:
        sw = pump(cw, 25)   # ~0.1s 유지
        print(f"[{name:12}] status=0x{sw:04X}   2403h = {rd2403()}")

    # 0x0F 유지하며 Operation Enabled 대기 + 2403h 추적
    print("\n0x0F 유지, Operation Enabled(bit2) 대기 중...")
    ok = False
    for i in range(1000):
        sw = io(0x0F)
        if sw & 0x0004:
            ok = True
            print(f"  ✅ Enabled at {int(i*0.004*1000)}ms  status=0x{sw:04X}  2403h={rd2403()}")
            break
        time.sleep(0.004)
    if not ok:
        print(f"  enable 실패 status=0x{io(0x0F):04X}")

    # enabled 상태에서 3초간 2403h 유지 확인
    print("\nEnabled 3초 유지하며 2403h 추적:")
    for t in range(3):
        pump(0x0F, 250)
        print(f"  {t+1}초  status=0x{io(0x0F):04X}  2403h = {rd2403()}")

    pump(0x0000, 10)
    master.state = pysoem.INIT_STATE; master.write_state()
    master.close()
    print("\n종료 — 2403h가 어느 단계에서든 1→0으로 떨어졌는지 확인하세요.")


if __name__ == "__main__":
    main()
