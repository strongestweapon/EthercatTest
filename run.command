#!/bin/bash
# TL-E EtherCAT 모터 GUI 실행기 (더블클릭 또는 터미널에서 실행)
# raw 이더넷 접근에 관리자 권한이 필요하므로 sudo로 실행됨 → 맥 로그인 암호 입력
cd "$(dirname "$0")"
PY="/Library/Frameworks/Python.framework/Versions/3.13/bin/python3"
echo "TL-E EtherCAT 모터 GUI 실행 (관리자 암호 필요)"
sudo "$PY" "$(dirname "$0")/tlc_ethercat_gui.py"
