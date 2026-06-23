# Network Diagram

```
subgraph Vision_Side
    Cam[USB/Web Camera] --> VPC[Vision Sub PC: vision_pkg]
end

subgraph Control_Side
    MPC[Main PC: robot_control_pkg ROS2 Logic + hmi_pkg HMI]
    RobotCtrl[doosan_robot_control_node]
    M0609[M0609 Robot Arm]
    Gripper[gripper_control_node]
end

VPC <--ROS2 DDS (LAN / Wi-Fi)--> MPC
MPC --> RobotCtrl
RobotCtrl --> M0609
M0609 --> Gripper
```

## 권장 네트워크 구조

1. Vision Sub PC와 Main PC는 동일한 네트워크 대역에 연결한다.
2. `ROS_DOMAIN_ID`를 양쪽 PC에 동일하게 설정한다 (예: `export ROS_DOMAIN_ID=30`).
3. 카메라 추론 결과(`/vision/tube_height`, `/vision/tube_state`)는 Vision PC에서 발행하고, Main PC에서 구독한다.
4. 로봇 제어 명령(`/robot/start_task`, `/robot/move_to_pose`, `/gripper/control`)은 Main PC에서 생성/호스팅한다.
5. 현재 구현에서는 모든 노드가 단일 PC에서 실행되도록 mock 되어 있다 (`ros2 launch launch/system_launch.py`). 2-PC 분리 시 `vision_pkg`는 Vision Sub PC에서, `robot_control_pkg`/`hmi_pkg`는 Main PC에서 각각 `ros2 launch`로 실행하면 된다 (코드 변경 불필요, DDS가 네트워크 너머로 토픽/서비스를 자동 매칭한다).
