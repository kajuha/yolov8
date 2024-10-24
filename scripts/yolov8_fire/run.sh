#!/bin/bash

# 경로 인자를 받음
# 이렇게 변경한 이유는 aliencontrol에서 source 및 python을 실행할 경우
# ros 경로 에러가 발생함
# 그래서 쉘스크립트로 python 변수영역이 분리될 것으로 생각함

cd $1
if [ ! -d "venv" ]
then
    echo "venv not exist."
    # 초기에는 3.6 정상사용, 3.8 테스트결과 이상없음
    # ros를 사용하기 위해 시스템의 ros패키지를 임포트하기 위함
    virtualenv --system-site-packages --python=python3 venv &&\
    source venv/bin/activate &&\
    pip install -r requirements.txt &&\
    deactivate
fi

if [ -z "$2" ]
then
    echo "No callback_hz supplied"
    exit -1
else
    CALLBACK_HZ=$2
    echo "callback_hz :" $CALLBACK_HZ
fi
source venv/bin/activate && python yolov8_fire.py --callback_hz $CALLBACK_HZ