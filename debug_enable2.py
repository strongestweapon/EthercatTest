#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""enable 심화: 전류설정 확인 + PDO/SDO 상태 교차검증 + enable 변형 시도."""
import sys, struct, time
import pysoem

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
SPEED = 5000
master = pysoem.Master()
sl = None


def dec(sw):
    b = [("RDY", 1), ("ON", 2), ("ENA", 4), ("FLT", 8), ("VOLT", 0x10),
         ("QS", 0x20), ("NRDY", 0x40), ("REM", 0x200)]
    return f"0x{sw:04X}[" + " ".join(n for n, m in b if sw & m) + "]"


def io(cw):
    sl.output = struct.pack("<HiI", cw & 0xFFFF, 0, SPEED)
    master.send_processdata(); master.receive_processdata(2000)
    return struct.unpack("<Hii", bytes(sl.input)[:10])[0]


def pump(cw, n):
    sw = 0
    for _ in range(n):
        sw = io(cw); time.sleep(0.004)
    return sw


def u(idx, sub=0, fmt="<H"):
    try:
        return struct.unpack(fmt, sl.sdo_read(idx, sub)[:struct.calcsize(fmt)])[0]
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

    print("===== 전류/모드 설정 =====")
    print(f"  2301h 루프모드(1개2폐)={u(0x2301)}")
    print(f"  2303h 피크전류(mA)={u(0x2303)}")
    print(f"  2304h 기본홀딩%={u(0x2304)}")
    print(f"  2305h 폐루프홀딩%={u(0x2305)}")
    print(f"  2306h 개루프홀딩%={u(0x2306)}")
    print(f"  2307h 락샤프트전류%={u(0x2307)}")
    print(f"  6502h 지원모드=0x{u(0x6502,0,'<I'):X}")
    print(f"  6060/6061 모드={u(0x6060,0,'b')}/{u(0x6061,0,'b')}")

    print("\n===== PDO vs SDO 상태 교차검증 (현재 cw=0x00) =====")
    pdo_sw = pump(0x00, 10)
    sdo_sw = u(0x6041)
    print(f"  PDO 6041={dec(pdo_sw)}   SDO 6041=0x{sdo_sw:04X}  (일치?{pdo_sw==sdo_sw})")

    print("\n===== enable 변형 시도 (각 시도 후 PDO+SDO 상태) =====")
    trials = [
        ("standard 06-07-0F", [0x06, 0x07, 0x0F]),
        ("직접 0F", [0x0F]),
        ("00 후 0F", [0x00, 0x0F]),
        ("reset 후 06-07-0F", [0x80, 0x00, 0x06, 0x07, 0x0F]),
        ("06-07-0F 2초hold", [0x06, 0x07]),
    ]
    for name, seq in trials:
        for cw in seq:
            sw = pump(cw, 15)
        if "2초" in name:
            sw = pump(0x0F, 500)   # 2초
        sdo = u(0x6041)
        print(f"  [{name:22s}] PDO={dec(sw)} SDO=0x{sdo:04X} err=0x{u(0x603F):04X}")
        pump(0x00, 5)

    pump(0x00, 5)
    master.state = pysoem.INIT_STATE; master.write_state()
    master.close()
    print("\n종료")


if __name__ == "__main__":
    main()
