# Network Diagram

## 물리 구성

```
┌─────────────────────────────────────────────────────────────────┐
│  Vision Sub PC  (vision_pkg)                                    │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  side_camera_node  →  liquid_height_detector_node          │ │
│  │                    →  tube_state_publisher_node             │ │
│  └────────────────────────────────────────────────────────────┘ │
│  USB  ↑                                                         │
│  Side-view Camera                                               │
└──────────────────────────────────┬──────────────────────────────┘
                                   │  ROS2 DDS (LAN/Wi-Fi)
                                   │  ROS_DOMAIN_ID=30
┌──────────────────────────────────┴──────────────────────────────┐
│  Main PC  (robot_control_pkg + hmi_pkg)                         │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  main_decision_node                                         │ │
│  │  robot_task_manager_node                                    │ │
│  │  doosan_robot_control_node  ── Doosan SDK ──────────────┐  │ │
│  │  gripper_control_node  ─── Modbus TCP ─────────────┐    │  │ │
│  │  hmi_node  (PyQt5 GUI)                              │    │  │ │
│  └─────────────────────────────────────────────────────│────│──┘ │
└────────────────────────────────────────────────────────│────│────┘
                                                         │    │
                         ┌───────────────────────────────┘    │
                         │  Modbus TCP  192.168.1.1:502        │
                         ▼                                     │
              ┌──────────────────┐         EtherCAT/LAN        │
              │  OnRobot RG2     │                             │
              │  Gripper         │◄────────────────────────────┘
              └──────────────────┘    Doosan DSR API (movel/movej)
                         │                    │
                         └────────────────────┘
                               M0609 로봇 암
```

## 통신 프로토콜 요약

| 구간 | 프로토콜 | 내용 |
|------|----------|------|
| Vision PC ↔ Main PC | ROS2 DDS (UDP Multicast) | Topic / Service |
| Main PC → M0609 | Doosan DSR API (`movel`, `movej`, `GetToolForce`) | 로봇 모션, 외력 측정 |
| Main PC → Gripper | Modbus TCP (raw socket, FC3, Reg 268) | OPEN/CLOSE, 상태 폴링 |
| Main PC → Gripper HW | Doosan digital I/O (`set_digital_output`) | 그리퍼 트리거 |

## 네트워크 설정

1. Vision Sub PC 와 Main PC를 동일 서브넷에 연결한다.
2. 양쪽 PC 모두 `export ROS_DOMAIN_ID=30` (또는 동일한 값).
3. Vision PC → `/vision/tube_state`, `/vision/tube_height`, `/vision/side_image` 발행.
4. Main PC → `/robot/status`, `/robot/force_norm`, `/robot/force_detected` 발행.
5. 단일 PC 실행 시: `ros2 launch launch/system_launch.py source_mode:=device camera_index:=4`
6. 2-PC 분리 시: vision_pkg는 Vision PC에서, robot_control_pkg / hmi_pkg는 Main PC에서 각각 `ros2 launch`. 코드 변경 없이 DDS가 토픽/서비스를 자동 매칭한다.
