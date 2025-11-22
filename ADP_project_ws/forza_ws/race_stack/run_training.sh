#!/bin/bash

# 1. 학습할 맵 리스트 (config/maps 폴더 내의 이름)
# (주의: map_name이 폴더명과 다르다면 _orig 등을 붙여줘야 할 수 있습니다)
MAPS=("small_hall" , "teras")

# 2. 각 맵당 학습 시간 (초)
DURATION=1800 # 30분

# 3. 워크스페이스 경로
WS_PATH="."

# ROS 2 환경 설정
source /opt/ros/humble/setup.bash
source source/install/setup.bash

echo "Starting Auto-Training Sequence..."

for map in "${MAPS[@]}"
do
    echo "------------------------------------------"
    echo ">>> Loading Map: $map"
    echo "------------------------------------------"

    # (A) 1단계: Base System 실행 (시뮬레이터 & 맵)
    # map_dir: 맵 폴더 이름 / map_name: .yaml 파일 이름 (보통 폴더명과 같음)
    # 만약 파일명이 '_orig'가 붙는 규칙이라면 map_name:=${map}_orig 로 수정하세요.
    ros2 launch stack_master base_system_launch.xml racecar_version:=OrinNano map_dir:=$map map_name:=${map}_orig sim:=true &
    PID_BASE=$!
    
    # 시뮬레이터가 완전히 켜질 때까지 대기
    sleep 5

    # (B) 2단계: Controller 실행 (TRAINING 모드)
    # ctrl_algo:=TRAINING 으로 설정하여 데이터 수집 모드 진입
    ros2 launch stack_master time_trials_launch.xml racecar_version:=OrinNano LU_table:=F1TENTH_Pacejka ctrl_algo:=TRAINING &
    PID_CTRL=$!
    
    # 컨트롤러 로딩 대기
    sleep 5

    # (C) 3단계: Training Supervisor 실행
    # Supervisor는 별도의 런치 파일 없이 노드를 직접 실행합니다.
    # (setup.py에 entry_point가 'training_supervisor = ...' 로 등록되어 있어야 합니다)
    ros2 run stack_master training_supervisor &
    PID_SUP=$!

    # (D) 학습 진행 대기
    echo ">>> Training for $DURATION seconds..."
    
    # 진행 상황 카운트다운
    for ((i=DURATION; i>0; i-=60)); do
        echo "Time remaining: $i seconds"
        sleep 60
    done

    # (E) 종료 및 정리 (역순으로 종료)
    echo ">>> Switching Map..."
    
    # 1. 감시자 종료
    kill -INT $PID_SUP
    # 2. 컨트롤러 종료 (데이터 저장 유도)
    kill -INT $PID_CTRL
    # 3. 시뮬레이터 종료
    kill -INT $PID_BASE
    
    # 프로세스가 완전히 죽을 때까지 대기
    sleep 10
    
    # 혹시 안 죽었으면 강제 종료 (좀비 프로세스 방지)
    kill -9 $PID_SUP 2>/dev/null
    kill -9 $PID_CTRL 2>/dev/null
    kill -9 $PID_BASE 2>/dev/null
    
    echo ">>> Cleanup Done."
    sleep 2
done

echo "=========================================="
echo "All Training Sessions Completed!"
echo "Data saved in .npz files."
echo "=========================================="