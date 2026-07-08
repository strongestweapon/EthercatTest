#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""드라이버에 실시간 전류/전압/토크/온도 읽기 오브젝트가 있는지 스캔."""
import sys, struct
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
master = pysoem.Master()

# 표준 CiA402 측정 후보
CAND = {
    0x6077: "Torque actual (표준)",
    0x6078: "Current actual (표준, ‰정격)",
    0x6079: "DC link voltage (표준, mV)",
    0x6064: "Position actual",
    0x606C: "Velocity actual",
    0x6041: "Statusword",
    0x603F: "Error code",
    0x2100: "InputIOStatus",
    0x2101: "OutputIOStatus",
    0x2001: "SW version",
}


def try_read(s, idx, sub=0):
    try:
        raw = s.sdo_read(idx, sub)
        return raw
    except Exception:
        return None


def main():
    master.open(IFACE)
    if master.config_init() <= 0:
        print("슬레이브 없음 (GUI 닫았는지 확인)")
        return
    s = master.slaves[0]

    print("=== 지정 후보 ===")
    for idx, name in CAND.items():
        raw = try_read(s, idx)
        if raw is None:
            print(f"  {idx:#06x} {name:28s} : (없음)")
        else:
            v16 = struct.unpack("<h", raw[:2])[0] if len(raw) >= 2 else None
            v32 = struct.unpack("<i", raw[:4])[0] if len(raw) >= 4 else None
            print(f"  {idx:#06x} {name:28s} : len={len(raw)} u16={struct.unpack('<H',raw[:2])[0] if len(raw)>=2 else '-'} i16={v16} i32={v32} hex={raw.hex()}")

    print("\n=== 제조사영역 스캔 0x2000~0x21FF (존재하는 것만) ===")
    for idx in range(0x2000, 0x2200):
        raw = try_read(s, idx)
        if raw is not None:
            u = struct.unpack("<H", raw[:2])[0] if len(raw) >= 2 else raw[0]
            print(f"  {idx:#06x} len={len(raw)} u16={u} hex={raw.hex()}")

    master.close()
    print("\n완료")


if __name__ == "__main__":
    main()
