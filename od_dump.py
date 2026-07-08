#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""드라이버 전체 오브젝트 사전(OD)을 SDO Information 서비스로 통째로 읽어 이름까지 나열.
전류/전압/토크/온도 명칭 오브젝트를 찾는다."""
import sys, struct
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
master = pysoem.Master()


def main():
    master.open(IFACE)
    if master.config_init() <= 0:
        print("슬레이브 없음 (GUI 닫았는지 확인)")
        return
    sl = master.slaves[0]

    print("OD 읽는 중 (SDO Information)... 조금 걸립니다")
    try:
        od = sl.od           # pysoem: SDO info로 전체 OD 읽기
    except Exception as e:
        print("read_od 실패:", e)
        master.close()
        return

    kw = ["current", "voltage", "volt", "torque", "temp", "bus", "dc",
          "电流", "电压", "温度", "母线", "전류", "전압"]
    print(f"총 오브젝트 {len(od)}개\n")
    hits = []
    for o in od:
        try:
            name = o.name
        except Exception:
            name = "?"
        line = f"  {o.index:#06x}  {name}"
        low = str(name).lower()
        if any(k in low for k in kw):
            hits.append(line)
        # 엔트리(서브인덱스) 이름도 검사
        try:
            for e in o.entries:
                en = str(getattr(e, "name", "")).lower()
                if en and any(k in en for k in kw):
                    hits.append(f"      └ sub {getattr(e,'name','')} @ {o.index:#06x}")
        except Exception:
            pass

    print("=== 전류/전압/토크/온도 관련 오브젝트 ===")
    if hits:
        for h in hits:
            print(h)
    else:
        print("  (이름에 해당 키워드 없음)")

    print("\n=== 전체 목록 ===")
    for o in od:
        try:
            print(f"  {o.index:#06x}  {o.name}")
        except Exception:
            print(f"  {o.index:#06x}  ?")

    master.close()


if __name__ == "__main__":
    main()
