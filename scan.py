#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EtherCAT 슬레이브 연결 확인용 스캔 (모터는 움직이지 않음)."""
import sys
import struct
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"

def rd(slave, idx, sub, fmt):
    try:
        raw = slave.sdo_read(idx, sub)
        return struct.unpack(fmt, raw[:struct.calcsize(fmt)])[0]
    except Exception as e:
        return f"(읽기실패:{e})"

def main():
    m = pysoem.Master()
    print(f"[i] '{IFACE}' 열기...")
    m.open(IFACE)
    n = m.config_init()
    if n <= 0:
        print("[ERR] 슬레이브 0개 — 전원/랜선/CN6A(Input)포트/인터페이스명 확인")
        m.close()
        return
    print(f"[i] ✅ 슬레이브 {n}개 발견\n")
    for i, s in enumerate(m.slaves):
        print(f"--- 슬레이브 #{i} ---")
        print(f"  name         : {s.name}")
        print(f"  ManufacturerID: {hex(s.man)}   (매뉴얼 기대값 0xa79)")
        print(f"  ProductCode  : {hex(s.id)}    (매뉴얼 기대값 0x1000)")
        print(f"  Revision     : {hex(s.rev)}")
        print(f"  현재 상태     : {hex(s.state)}")
        # 식별정보 (0x1018) / 장치명 / 지원 운전모드
        print(f"  DeviceName(1008h)     : {rd(s, 0x1008, 0, '<H')}")
        print(f"  SupportedModes(6502h) : {rd(s, 0x6502, 0, '<I')}  (bit0=PP bit2=PV bit5=HM bit7=CSP)")
        print(f"  운전모드(6060h)        : {rd(s, 0x6060, 0, 'b')}")
        print(f"  DriveOperatingMode(2301h,1:개루프 2:폐루프): {rd(s, 0x2301, 0, '<H')}")
        print(f"  Subdivision(2302h,펄스/회전): {rd(s, 0x2302, 0, '<H')}")
        print(f"  StatusWord(6041h)     : {hex(rd(s, 0x6041, 0, '<H')) if isinstance(rd(s,0x6041,0,'<H'),int) else rd(s,0x6041,0,'<H')}")
        print(f"  ErrorCode(603Fh)      : {hex(rd(s, 0x603F, 0, '<H')) if isinstance(rd(s,0x603F,0,'<H'),int) else rd(s,0x603F,0,'<H')}")
    m.close()
    print("\n[i] 스캔 완료 (모터는 움직이지 않음)")

if __name__ == "__main__":
    main()
