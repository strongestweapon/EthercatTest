#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""브레이크 기능 무장(arm) + EEPROM 저장.
매뉴얼 3.6: IO/브레이크 파라미터는 '저장 + 재시작'해야 반영됨.
  1) 2403h Brake Enable = 1
  2) 2201h Parameter Save 0->1 엣지 트리거 (공장 파라미터 저장)
실행 후 반드시 드라이버 전원 재투입(power cycle) → 그 다음 brake_hold.py enable 로 측정.
사용:  sudo python3 brake_arm.py [en7]
"""
import sys, struct, time
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
master = pysoem.Master()


def rd(sl, idx, sub=0):
    try:
        return struct.unpack("<H", sl.sdo_read(idx, sub)[:2])[0]
    except Exception as e:
        return f"err:{e}"


def main():
    master.open(IFACE)
    if master.config_init() <= 0:
        print("슬레이브 없음 (GUI 앱 닫았는지 확인)"); return
    sl = master.slaves[0]
    # SDO는 PreOP에서도 동작. PDO 매핑/OP 불필요.
    master.state = pysoem.PREOP_STATE
    master.write_state()
    master.state_check(pysoem.PREOP_STATE, 50000)

    print("=== 저장 전 ===")
    print(f"  2403h Brake Enable = {rd(sl, 0x2403)}  (0=Disabled, 1=Enabled)")
    print(f"  2404h Open Delay   = {rd(sl, 0x2404)} ms")
    print(f"  2405h Close Delay  = {rd(sl, 0x2405)} ms")

    print("\n[1] 2403h = 1 (브레이크 기능 ON)")
    sl.sdo_write(0x2403, 0, struct.pack("<H", 1))
    time.sleep(0.1)
    v = rd(sl, 0x2403)
    print(f"    되읽기 2403h = {v}")
    if v != 1:
        print("    ★ 2403h가 1로 안 써짐 — 중단");
        master.close(); return

    print("[2] 2201h Parameter Save  (0 -> 1 엣지 트리거)")
    sl.sdo_write(0x2201, 0, struct.pack("<H", 0))
    time.sleep(0.1)
    sl.sdo_write(0x2201, 0, struct.pack("<H", 1))
    time.sleep(0.5)   # 내부 EEPROM 기록 대기
    print(f"    2201h 되읽기 = {rd(sl, 0x2201)}")
    print(f"    2403h 재확인 = {rd(sl, 0x2403)}")

    master.state = pysoem.INIT_STATE
    master.write_state()
    master.close()
    print("\n===============================================")
    print(" ★★ 지금 드라이버 전원을 껐다 켜세요 (power cycle) ★★")
    print(" 재부팅 후:  sudo python3 brake_hold.py enable 30")
    print("   → enable 상태에서 GND→BK+ 전압 측정")
    print("===============================================")


if __name__ == "__main__":
    main()
