# Vision AI 기반 시약 품질 판정 및 협동로봇 자동 분류 시스템

**Vision-AI-Based Reagent Quality Inspection and Collaborative Robot Sorting System**

카메라 기반 시약 높이 추론 결과를 ROS2 통신으로 전달하고, 두산로보틱스 M0609 협동로봇이 시약통 상태에 따라 보정, 폐기, 정상 이송 작업을 수행하는 실험실 자동화 시스템.

## 1. Project Overview

| 구성 요소 | 역할 | 패키지 |
|---|---|---|
| Vision Sub PC | 카메라 영상 입력, YOLOv8n 기반 시약 높이 추론 | `vision_pkg` |
| Main PC | ROS2 통신 수신, 상태 판단, 로봇 명령 생성, HMI 실행 | `robot_control_pkg`, `hmi_pkg` |
| Doosan M0609 | 시약 보충, 폐기 이송, 정상 트레이 이송 수행 (현재는 mock) | `robot_control_pkg` |
| Camera System | Side-view 기반 3x1 시약통 높이 측정 | `vision_pkg` |
| Interfaces | 커스텀 ROS2 메시지/서비스 정의 | `interfaces` |

## 2. Background

실험실에서는 시약 혼합, 분주, 폐기, 정리 과정에서 반복 작업과 화학물질 취급 위험이 동시에 발생한다. 액체량 오류, 시약 누락, 잘못된 높이, 폐액 처리 실수 등은 실험 품질 저하뿐 아니라 안전사고로 이어질 수 있다. 본 프로젝트는 카메라 기반 비전 품질관리 시스템을 협동로봇과 결합하여, 단순 검출을 넘어 **시약 상태 판정 이후 로봇이 실제 후속 조치를 수행**하는 자동화 파이프라인을 구현한다.

## 3. System Architecture

```
Camera[Side-view Camera] --> VisionPC[Vision Sub PC]
VisionPC --|ROS2 Topic: /vision/tube_state|--> MainPC[Main PC]
MainPC --> HMI[HMI Dashboard]
MainPC --|Robot Command|--> Robot[M0609 Collaborative Robot]
Robot --> Gripper[Gripper]
Robot --> ReagentZone[Reagent Fill Zone]
Robot --> WasteZone[Waste Zone]
Robot --> NormalZone[Normal Tray Zone]
```

세부 구성도: [docs/system_architecture.md](docs/system_architecture.md)
네트워크 구성: [docs/network_diagram.md](docs/network_diagram.md)

## 4. Hardware Setup (목표 사양)

- Side-view USB/웹 카메라 1대 (3x1 트레이 측면 촬영)
- Vision Sub PC: GPU 또는 CPU 추론 가능한 PC (YOLOv8n)
- Main PC: ROS2 Humble 실행, HMI 표시
- Doosan Robotics M0609 협동로봇 + 그리퍼 (현재 구현은 mock; 실제 연동 시 `doosan_robot2`/`DSR_ROBOT2` API로 교체)

## 5. Software Stack

- ROS2 Humble, Python 3.10
- `ultralytics` (YOLOv8n, 파인튜닝된 3-class 모델: `cup`/`height`/`hand`)
- OpenCV (카메라 캡처), NumPy (이미지 변환 - cv_bridge 미사용, 아래 참고)
- Tkinter + Pillow (HMI 대시보드)
- colcon / ament_python / ament_cmake (빌드)

> **참고**: 이 환경의 `cv_bridge`(apt) 바이너리가 pip로 설치된 numpy 2.x와 ABI가 맞지 않아 세그폴트가 발생하여, `vision_pkg`는 `cv_bridge` 대신 `vision_pkg/ros_image_utils.py`에서 `sensor_msgs/Image`(bgr8) <-> numpy 배열 변환을 직접 구현해 우회한다.

## 6. ROS2 Node Graph

```
side_camera_node --> liquid_height_detector_node --> tube_state_publisher_node --> main_decision_node
main_decision_node --> robot_task_manager_node --> doosan_robot_control_node --> gripper_control_node
tube_state_publisher_node, liquid_height_detector_node, main_decision_node --> hmi_node (구독만)
```

| Node | 패키지 | 역할 |
|---|---|---|
| `side_camera_node` | vision_pkg | Side-view 카메라(또는 테스트 이미지 디렉토리) 이미지 취득, `/vision/side_image` 발행 |
| `liquid_height_detector_node` | vision_pkg | YOLOv8n(cup/height/hand) 추론, 좌→우 3구역 매핑, 액체 높이 calibration, `/vision/tube_height`/`/vision/hand_detected`/`/vision/camera_status` 발행 |
| `tube_state_publisher_node` | vision_pkg | 임계값 기반 State 0/1/2/UNKNOWN 분류, `/vision/tube_state` 발행, `/vision/request_recheck` 서비스 호스팅 |
| `main_decision_node` | robot_control_pkg | `/vision/tube_state` 구독, 안전 게이트 확인 후 `/robot/start_task` 호출 (반응형 단일 흐름) |
| `robot_task_manager_node` | robot_control_pkg | `/robot/start_task`, `/robot/stop_task` 호스팅, 우선순위(State2>State0>State1) 기반 작업 시퀀스 실행 |
| `doosan_robot_control_node` | robot_control_pkg | `/robot/move_to_pose` 호스팅 (mock 팔 이동, 향후 `DSR_ROBOT2.movel`로 교체) |
| `gripper_control_node` | robot_control_pkg | `/gripper/control` 호스팅 (mock 그리퍼 개폐, 향후 OnRobot RG API로 교체) |
| `hmi_node` | hmi_pkg | Tkinter 대시보드: 카메라뷰/상태패널/제어패널/로그/안전패널 |

전체 노드 그래프 문서: [docs/ros2_node_graph.md](docs/ros2_node_graph.md)

### Topic / Service 정의

| 종류 | 이름 | 타입 | 설명 |
|---|---|---|---|
| Topic | `/vision/side_image` | `sensor_msgs/Image` | side-view 원본 프레임 |
| Topic | `/vision/tube_height` | `interfaces/TubeHeight` | 시약통별 추론 높이/신뢰도 |
| Topic | `/vision/tube_state` | `interfaces/TubeState` | 시약통별 State(0/1/2/-1) |
| Topic | `/vision/hand_detected` | `std_msgs/Bool` | 작업영역 내 손 감지 (안전, PDF 보충 설계) |
| Topic | `/vision/camera_status` | `std_msgs/Bool` | 카메라 프레임 수신 정상 여부 (안전, PDF 보충 설계) |
| Topic | `/robot/status` | `interfaces/RobotStatus` | 로봇 현재 상태(IDLE/MOVING/.../EMERGENCY_STOP) |
| Service | `/robot/start_task` | `interfaces/StartTask` | 상태 배열 기반 작업 시작 요청 |
| Service | `/robot/stop_task` | `interfaces/StopTask` | 비상 정지/작업 중단 요청 |
| Service | `/vision/request_recheck` | `interfaces/RequestRecheck` | 보충/폐기 후 재검사 요청 |
| Service | `/robot/move_to_pose` | `interfaces/MoveToPose` | (보충 설계) named pose로 팔 이동 |
| Service | `/gripper/control` | `interfaces/GripperControl` | (보충 설계) 그리퍼 OPEN/CLOSE |

`/vision/hand_detected`, `/vision/camera_status`, `/robot/move_to_pose`, `/gripper/control`은 원본 명세서(섹션 12)에 명시되지 않았지만, 노드 그래프(섹션 11)의 연결 관계와 안전 설계(섹션 16-17)를 실제로 구현하기 위해 추가한 보충 인터페이스다.

## 7. State Definition

| State | 의미 | 로봇 동작 | 색상(HMI) |
|---|---|---|---|
| 0 | 보충 필요 | 해당 튜브로 이동 → 보충 위치 접근 → 시약 추가 → 재검사 요청 | Yellow |
| 1 | 정상 | 개별 조치 없음. 3개 모두 1이면 트레이 전체를 정상 구역으로 이송 | Green |
| 2 | 폐기 필요 | 해당 튜브 파지 → 폐기 구역 이송 → 원위치 복귀 | Red |
| -1 | UNKNOWN (보충 설계) | confidence 낮음/검출 실패 시. 작업 보류 | Gray |

분류 로직(`vision_pkg/tube_state_publisher_node.py`)은 명세서 섹션 7의 임계값 로직을 그대로 구현한다:

```python
if refill_min <= height < target_height - tolerance:
    state = 0
elif target_height - tolerance <= height <= target_height + tolerance:
    state = 1
else:
    state = 2
```

우선순위(여러 State가 동시에 존재할 때, 명세서 섹션 8 Scenario D): **State 2 폐기 > State 0 보충 > State 1 정상 이송**.

## 8. Operation Scenario

`main_decision_node`가 `/vision/tube_state`를 받을 때마다 반응적으로 `robot_task_manager_node`의 `/robot/start_task`를 호출하고, `robot_task_manager_node`는 **그 순간 가장 높은 우선순위의 작업 한 개만 수행한 뒤 재검사를 요청**한다. 재검사로 갱신된 다음 `tube_state`가 다시 `main_decision_node`로 들어와 다음 작업을 결정하는 식으로, 여러 사이클에 걸쳐 Scenario A/B/C/D가 자연스럽게 완성된다 (명세서 섹션 14 순서도의 "Recheck → Capture로 회귀" 구조와 동일).

전체 동작 순서: [docs/operation_flow.md](docs/operation_flow.md)

## 9. HMI Design

`hmi_pkg/hmi_node.py` (Tkinter): Camera View / Tube State Panel(색상+높이) / Robot Status Panel / Control Panel(Start, Stop, Recheck, EMERGENCY STOP) / Log Panel / Safety Panel(카메라 상태, 손 감지). rclpy는 백그라운드 스레드에서 spin하고 Tkinter는 메인 스레드에서 150ms 주기로 캐시된 상태를 polling한다.

## 10. Safety Management

- 카메라 프레임이 `camera_timeout_sec` 이상 끊기면 `/vision/camera_status=False` 발행 → `main_decision_node`가 로봇 동작을 보류한다.
- `hand` 클래스가 검출되면 `/vision/hand_detected=True` 발행 → `main_decision_node`가 즉시 `/robot/stop_task`를 호출해 진행 중인 작업을 중단시키고, 손이 사라지기 전까지 새 작업을 보류한다.
- confidence가 임계값 미만이면 해당 튜브를 UNKNOWN(-1)으로 분류하여 작업 대상에서 제외한다.
- 그리퍼 파지 실패 시 1회 재시도(`grip_retry_count`) 후에도 실패하면 작업을 실패로 종료하고 `/robot/status`를 ERROR로 발행한다.
- 모든 좌표는 접근(approach) 위치와 작업(pick/refill) 위치로 분리되어 있어, 항상 안전 높이를 거쳐 하강/상승한다.

상세: [docs/safety_plan.md](docs/safety_plan.md)

## 11. Demo

처음 클론한 환경에서 설정 없이 실행하는 방법은 **[SETUP.md](SETUP.md)** 참고. 요약:

```bash
pip3 install -r requirements.txt
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch launch/system_launch.py            # vision + robot_control + hmi
ros2 launch launch/system_launch.py launch_hmi:=false   # HMI 없이
```

`vision_pkg`에 YOLOv8n weight(`weights/reagent_yolov8n.pt`)와 hand 없는 샘플 이미지 10장(`sample_images/`)이 번들되어 있어, 추가 설정/외부 데이터셋 없이도 실제 YOLOv8n 추론 전체 파이프라인을 바로 시연할 수 있다. 실제 카메라/모델 사용법은 SETUP.md의 6번 항목 참고 (`source_mode:=device camera_index:=0`, `model_path:=...` 등 launch argument로 전달).

## 12. Team Members

- jiwan (gwanshin12301230@gmail.com)

## Folder Structure

```
project-root/
├── README.md
├── SETUP.md                 # 환경 구성 가이드 (clone부터 실행까지)
├── requirements.txt          # pip 의존성 (ultralytics, opencv-python, numpy, Pillow)
├── docs/
│   ├── system_architecture.md
│   ├── network_diagram.md
│   ├── ros2_node_graph.md
│   ├── operation_flow.md
│   └── safety_plan.md
├── src/
│   ├── interfaces/          # custom msg/srv (ament_cmake)
│   ├── vision_pkg/          # camera + YOLOv8n detector + state publisher
│   │   ├── weights/         # 번들된 YOLOv8n weight (cup/height/hand, 6MB)
│   │   └── sample_images/   # 카메라 없이 테스트할 샘플 이미지 10장
│   ├── robot_control_pkg/   # decision logic + mock Doosan M0609 control
│   └── hmi_pkg/             # Tkinter operator dashboard
├── assets/
│   ├── images/
│   ├── diagrams/
│   └── demo/
└── launch/
    └── system_launch.py
```

## Future Work

- Top-view 3x3 트레이 검사 확장 (명세서 섹션 18)
- `doosan_robot_control_node`/`gripper_control_node`를 실제 `doosan_robot2`(`DSR_ROBOT2.movel`/`mwait`) 및 OnRobot RG 그리퍼 API로 교체
- 카메라 calibration 정밀화 (현재는 cup/height bbox 비율 기반 단순 calibration)
