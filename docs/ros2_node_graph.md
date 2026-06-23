# ROS2 Node Graph

```
side_camera_node --> liquid_height_detector_node --> tube_state_publisher_node --> main_decision_node
main_decision_node --> robot_task_manager_node --> doosan_robot_control_node
robot_task_manager_node --> gripper_control_node
tube_state_publisher_node --> hmi_node (구독)
liquid_height_detector_node --> hmi_node (구독: side_image, hand_detected, camera_status)
robot_task_manager_node --> hmi_node (구독: robot/status)
robot_task_manager_node --> tube_state_publisher_node (/vision/request_recheck 호출)
```

## Node 역할

| Node | 패키지 | 역할 |
|---|---|---|
| `side_camera_node` | vision_pkg | Side-view 카메라(또는 테스트 이미지 디렉토리/비디오) 이미지 취득, `/vision/side_image` 발행 |
| `liquid_height_detector_node` | vision_pkg | YOLOv8n(cup/height/hand) 기반 추론. cup bbox를 좌/중/우 3구역에 매핑(`assign_tube_zones`), 매칭되는 height bbox와의 비율로 액체 높이 calibration(`liquid_fill_fraction`). `/vision/tube_height`, `/vision/hand_detected`, `/vision/camera_status` 발행 |
| `tube_state_publisher_node` | vision_pkg | 임계값 로직으로 State 0/1/2/UNKNOWN 분류, `/vision/tube_state` 발행. `/vision/request_recheck` 서비스 호스팅 |
| `main_decision_node` | robot_control_pkg | `/vision/tube_state` + 안전 토픽 구독, 안전 게이트 통과 시 `/robot/start_task` 호출. busy-flag로 중복 호출 방지, "전체 정상" 트레이는 1회만 이송 트리거 |
| `robot_task_manager_node` | robot_control_pkg | `/robot/start_task`, `/robot/stop_task` 호스팅. 우선순위(State2>State0>State1) 중 가장 높은 1개 작업만 수행 후 recheck 요청. `/robot/status` 발행 |
| `doosan_robot_control_node` | robot_control_pkg | `/robot/move_to_pose` 호스팅. named pose(`config/robot_params.yaml`)로 이동을 mock 시뮬레이션 |
| `gripper_control_node` | robot_control_pkg | `/gripper/control` 호스팅. OPEN/CLOSE를 mock 시뮬레이션 |
| `hmi_node` | hmi_pkg | 모든 상태 토픽 구독 + 3개 서비스 클라이언트(start_task/stop_task/request_recheck)로 수동 제어 제공 |

## Topic / Service / Action 정의

### Topics

| Topic | 타입 | 발행 노드 | 설명 |
|---|---|---|---|
| `/vision/side_image` | `sensor_msgs/Image` (bgr8) | side_camera_node | 원본 프레임 |
| `/vision/tube_height` | `interfaces/TubeHeight` | liquid_height_detector_node | `tube_index[]`, `liquid_height[]`(mm), `confidence[]` |
| `/vision/tube_state` | `interfaces/TubeState` | tube_state_publisher_node | `tube_index[]`, `state[]` (0/1/2/-1) |
| `/vision/hand_detected` | `std_msgs/Bool` | liquid_height_detector_node | 작업영역 내 손 검출 (보충 설계) |
| `/vision/camera_status` | `std_msgs/Bool` | liquid_height_detector_node | 카메라 프레임 수신 정상 여부 (보충 설계) |
| `/robot/status` | `interfaces/RobotStatus` | robot_task_manager_node | `status`(IDLE/MOVING/REFILLING/PICKING/DISPOSING/TRANSFER_NORMAL/ERROR/EMERGENCY_STOP), `current_task`, `detail` |

### Services

| Service | 타입 | 호스트 노드 | 설명 |
|---|---|---|---|
| `/robot/start_task` | `interfaces/StartTask` | robot_task_manager_node | `tube_index[]`, `state[]` → `success`, `message` |
| `/robot/stop_task` | `interfaces/StopTask` | robot_task_manager_node | `stop` → `success`, `message` (비상정지) |
| `/vision/request_recheck` | `interfaces/RequestRecheck` | tube_state_publisher_node | `request` → 다음 `/vision/tube_state` 발행까지 대기 후 `success`, `message` |
| `/robot/move_to_pose` | `interfaces/MoveToPose` (보충 설계) | doosan_robot_control_node | `pose_name` → `success`, `message` |
| `/gripper/control` | `interfaces/GripperControl` (보충 설계) | gripper_control_node | `command`(OPEN/CLOSE) → `success`, `message` |

`/vision/hand_detected`, `/vision/camera_status`, `/robot/move_to_pose`, `/gripper/control`은 명세서 섹션 12에는 없으나, 섹션 11 노드 그래프(`robot_task_manager_node --> doosan_robot_control_node --> gripper_control_node`)와 섹션 16-17 안전 설계를 실제로 동작시키기 위해 추가한 보충 인터페이스다.

## 좌표 매핑 (섹션 13)

`robot_control_pkg/robot_control_pkg/poses.py`에 이름 카탈로그, `config/robot_params.yaml`에 실제 `[x, y, z, rx, ry, rz]` 값(Doosan posx 규약)을 정의한다.

| Index | 위치 | approach pose | pick/refill pose |
|---|---|---|---|
| 0 | 왼쪽 시약통 | `tube_0_approach_pose` | `tube_0_pick_pose` / `tube_0_refill_pose` |
| 1 | 가운데 시약통 | `tube_1_approach_pose` | `tube_1_pick_pose` / `tube_1_refill_pose` |
| 2 | 오른쪽 시약통 | `tube_2_approach_pose` | `tube_2_pick_pose` / `tube_2_refill_pose` |

작업 구역: `waste_pose`, `normal_tray_approach_pose` / `normal_tray_pose`, `tray_pick_pose`(트레이 전체 파지), `home_pose`(대기 위치).
