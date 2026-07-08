# EthercatTest

맥(Mac)에서 **Lichuan TLC86E-48V-8.5** (NEMA34 통합형 폐루프 스텝모터, EtherCAT)를 pysoem(SOEM)으로 직접 구동한 프로젝트.

## 내용
- **[📄 상세 보고서 → report/2026-07-08.md](report/2026-07-08.md)** — 전체 과정·디버깅·기술 레퍼런스
- **`tlc_ethercat_gui.py`** — 모터 제어 GUI 앱
- **`run.command`** — 더블클릭 실행기 (sudo)
- `scan.py` / `e2e.py` / `demo.py` / `od_dump.py` — 진단·데모·오브젝트 덤프

## 빠른 시작
```bash
pip3 install pysoem
sudo python3 tlc_ethercat_gui.py     # raw 이더넷 접근에 sudo 필요
```
연결(`en7`) → 브레이크 해제 → 운전 ON → 조그 이동

## 핵심 요약
| | |
|---|---|
| 마스터 | 맥 + pysoem(SOEM), Free-Run, USB-C 이더넷 어댑터 |
| 프로토콜 | CoE / CiA402 (PP 모드) |
| 주의 | Enable은 폐루프 서보온에 ~1초 (상태워드 폴링) · **이동 전 브레이크 완전 해제 필수** |

![app](report/app_screenshot.png)
