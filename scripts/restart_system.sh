#!/bin/bash
# 시스템을 깔끔하게 재시작하는 스크립트.
# system_launch를 그냥 Ctrl+C 후 재시작하면 zombie 노드나 DDS 캐시 문제가
# 생길 수 있으므로, 이 스크립트를 통해 재시작하는 것을 권장한다.

set -e

echo "[restart] 기존 노드 종료 중..."

# 우리 패키지 노드들 + Doosan 하드웨어 드라이버를 이름으로 kill.
# ros2_control_node가 좀비로 남으면 재시작 시 DDS 엔드포인트가 꼬여
# set_singularity_handling 타임아웃이 발생하므로 반드시 포함한다.
pkill -SIGINT -f "robot_task_manager_node"    2>/dev/null || true
pkill -SIGINT -f "main_decision_node"         2>/dev/null || true
pkill -SIGINT -f "doosan_robot_control_node"  2>/dev/null || true
pkill -SIGINT -f "gripper_control_node"       2>/dev/null || true
pkill -SIGINT -f "liquid_height_detector_node" 2>/dev/null || true
pkill -SIGINT -f "tube_state_publisher_node"  2>/dev/null || true
pkill -SIGINT -f "side_camera_node"           2>/dev/null || true
pkill -SIGINT -f "hmi_node"                   2>/dev/null || true
pkill -SIGINT -f "ros2_control_node"          2>/dev/null || true
pkill -SIGINT -f "dsr_controller"             2>/dev/null || true

# SIGINT로 안 죽으면 3초 후 SIGTERM
sleep 3
pkill -SIGTERM -f "robot_task_manager_node"    2>/dev/null || true
pkill -SIGTERM -f "main_decision_node"         2>/dev/null || true
pkill -SIGTERM -f "doosan_robot_control_node"  2>/dev/null || true
pkill -SIGTERM -f "gripper_control_node"       2>/dev/null || true
pkill -SIGTERM -f "liquid_height_detector_node" 2>/dev/null || true
pkill -SIGTERM -f "tube_state_publisher_node"  2>/dev/null || true
pkill -SIGTERM -f "side_camera_node"           2>/dev/null || true
pkill -SIGTERM -f "hmi_node"                   2>/dev/null || true
pkill -SIGTERM -f "ros2_control_node"          2>/dev/null || true
pkill -SIGTERM -f "dsr_controller"             2>/dev/null || true

sleep 1

# ROS2 daemon 캐시 리셋 (stale 서비스/토픽 발견 정보 제거)
echo "[restart] ROS2 daemon 캐시 리셋..."
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start 2>/dev/null || true

echo "[restart] 재시작 완료. system_launch를 실행하세요."
echo ""
echo "  ros2 launch launch/system_launch.py source_mode:=device camera_index:=4"
