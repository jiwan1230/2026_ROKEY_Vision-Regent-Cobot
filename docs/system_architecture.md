# System Architecture

## 구성 요소

| 구성 요소 | 역할 | 구현 패키지 |
|---|---|---|
| Vision Sub PC | 카메라 영상 입력, 시약 높이 추론 수행 | `vision_pkg` |
| Main PC | ROS2 통신 수신, 상태 판단, 로봇 명령 생성, HMI 실행 | `robot_control_pkg`, `hmi_pkg` |
| Doosan M0609 | 시약 보충, 폐기 이송, 정상 트레이 이송 수행 | `robot_control_pkg` (현재 mock) |
| Camera System | Side-view 기반 3x1 시약통 높이 측정 | `vision_pkg` |

## 데이터 흐름

```
Camera[Side-view Camera]
  --> VisionPC[Vision Sub PC: side_camera_node]
  --> liquid_height_detector_node (YOLOv8n: cup/height/hand)
  --> tube_state_publisher_node (State 0/1/2/UNKNOWN 분류)
  --|ROS2 Topic: /vision/tube_state|--> MainPC[Main PC: main_decision_node]
  --> robot_task_manager_node
  --> Robot[M0609 Collaborative Robot] (doosan_robot_control_node + gripper_control_node)

MainPC --> HMI[HMI Dashboard: hmi_node]
Robot --> Gripper[Gripper]
Robot --> ReagentZone[Reagent Fill Zone: tube_X_refill_pose]
Robot --> WasteZone[Waste Zone: waste_pose]
Robot --> NormalZone[Normal Tray Zone: normal_tray_pose]
```

## 핵심 차별점

단순 이미지 판별에서 그치지 않고, 판단 결과를 기반으로 협동로봇이 실제 후속 조치를 수행하는 하나의 자동화 파이프라인:

```
Camera Perception
→ Liquid Height Estimation
→ State Classification
→ ROS2 Communication
→ Robot Task Planning
→ Refill / Disposal / Normal Transfer
→ Recheck
```

이 흐름은 `main_decision_node`(반응형 트리거) ↔ `robot_task_manager_node`(우선순위 1개 작업 수행 후 recheck 요청)의 사이클이 반복되며 구현된다. 즉 한 번의 서비스 호출이 전체 시나리오를 끝내는 것이 아니라, recheck로 갱신된 `/vision/tube_state`가 다음 사이클의 입력이 되어 점진적으로 전체 트레이를 정상 상태로 수렴시킨다.
