# Vision AI 기반 시약 자동 관리 협동로봇 시스템

**Vision-AI-Based Reagent Auto-Management Collaborative Robot System**

카메라로 시약 잔량을 추론하고, 두산로보틱스 M0609 협동로봇이 상태에 따라 **보충 / 폐기 / 정상 이송**을 자동으로 수행하는 실험실 자동화 파이프라인.

---

## 1. 시스템 개요

| 구성 요소 | 역할 | 패키지 |
|---|---|---|
| Vision Sub PC | 카메라 영상 입력, YOLOv8n 기반 시약 높이 추론 | `vision_pkg` |
| Main PC | ROS2 상태 판단, 로봇 명령 생성, HMI 실행 | `robot_control_pkg`, `hmi_pkg` |
| Doosan M0609 | 시약 보충, 폐기 이송, 정상 트레이 이송 | `robot_control_pkg` |
| Camera System | Side-view 기반 3×1 시약통 높이 측정 | `vision_pkg` |
| Interfaces | 커스텀 ROS2 메시지 / 서비스 정의 | `interfaces` |

```
카메라 → YOLO 추론 → 상태 분류 → ROS2 통신 → 로봇 제어
                                              ↕
                                         HMI (PyQt5)
```

### 시약통 상태 정의

| State | 의미 | 로봇 동작 |
|---|---|---|
| **0** | 시약 부족 | 보충 (Refill) |
| **1** | 정상 | 트레이 이송 (Normal Transfer, 전체 정상 시) |
| **2** | 과잉 / 불량 | 폐기 (Dispose) |
| **-1** | UNKNOWN | 신뢰도 미달 — 작업 보류, HMI 알림 |

우선순위: **State 2 (폐기) > State 0 (보충) > State 1 (이송)**

---

## 2. 개발 배경

실험실 시약 관리는 반복 수작업으로 개인 오차와 화학물질 취급 위험이 동시에 발생한다. 본 프로젝트는 카메라 기반 AI 품질관리와 협동로봇을 결합해 **단순 검출에서 그치지 않고, 판정 결과에 따라 로봇이 즉시 후속 조치를 수행**하는 완전한 자동화 파이프라인을 구현한다.

---

## 3. 시스템 아키텍처

```
Side Camera
    │ /vision/side_image (JPEG Compressed)
    ▼
LiquidHeightDetectorNode (YOLOv8n: cup / height / hand)
    │ /vision/tube_height        /vision/yolo_result_image
    │ /vision/hand_detected      /vision/camera_status
    ▼
TubeStatePublisherNode
    │ /vision/tube_state (State 0 / 1 / 2 / UNKNOWN)
    ▼
MainDecisionNode ←─ /robot/set_system_running (HMI Start/Stop)
    │ /robot/start_task
    ▼
RobotTaskManagerNode
    │ /robot/status    /robot/force_detected    /robot/force_norm
    ├─► DoosanRobotControlNode ──► M0609 (DSR API: movel, MoveStop, GetToolForce)
    └─► GripperControlNode (Modbus TCP)
    ▼
HMI Dashboard (PyQt5)
```

세부 구성도: [docs/system_architecture.md](docs/system_architecture.md)  
ROS2 노드 그래프: [docs/ros2_node_graph.md](docs/ros2_node_graph.md)

---

## 4. 하드웨어 구성

- **협동로봇**: Doosan Robotics M0609 (반경 900mm, 6-DOF)
- **카메라**: Side-view USB 카메라 1대 (3×1 트레이 측면 촬영)
- **Vision PC**: YOLOv8n CPU/GPU 추론 가능한 노트북
- **Main PC**: ROS2 Humble 실행 + PyQt5 HMI 표시

---

## 5. 소프트웨어 스택

| 항목 | 기술 |
|---|---|
| OS | Ubuntu 22.04 |
| Robot Framework | ROS2 Humble + DSR_MSGS2 |
| 언어 | Python 3.10 |
| Vision AI | YOLOv8n (Ultralytics), OpenCV |
| HMI | PyQt5 |
| 그리퍼 통신 | Modbus TCP |
| 빌드 | colcon, ament_python, ament_cmake |

> **cv_bridge 미사용**: pip numpy 2.x ↔ apt cv_bridge ABI 충돌로 세그폴트 발생.  
> `vision_pkg/ros_image_utils.py`에서 numpy로 이미지 변환을 직접 구현해 우회한다.

---

## 6. ROS2 노드 구성

| 노드 | 패키지 | 역할 |
|---|---|---|
| `side_camera_node` | vision_pkg | USB 카메라 / 테스트 이미지 캡처 → `/vision/side_image` (JPEG 압축) 발행 |
| `liquid_height_detector_node` | vision_pkg | YOLOv8n(cup/height/hand) 추론, Slot Anchor 기반 슬롯 매핑, 높이 % 계산, 손 감지 안전 신호 발행 |
| `tube_state_publisher_node` | vision_pkg | 임계값 기반 State 0/1/2/UNKNOWN 분류, `/vision/request_recheck` 서비스 호스팅 |
| `main_decision_node` | robot_control_pkg | `/vision/tube_state` 구독 → 안전 게이트 → `/robot/start_task` 호출 |
| `robot_task_manager_node` | robot_control_pkg | 우선순위(2>0>1) 작업 시퀀스 실행, `/robot/start_task` / `/robot/stop_task` 호스팅 |
| `doosan_robot_control_node` | robot_control_pkg | DSR API 기반 M0609 제어 (movel, MoveStop, GetToolForce, 가상 TCP) |
| `gripper_control_node` | robot_control_pkg | Modbus TCP 그리퍼 제어 (완료 신호 확인 포함) |
| `hmi_node` | hmi_pkg | PyQt5 대시보드 (카메라뷰 / 상태 / 로그 / JOG / 관리자 탭) |

### 주요 Topic / Service

| 종류 | 이름 | 타입 | 설명 |
|---|---|---|---|
| Topic | `/vision/side_image` | `sensor_msgs/CompressedImage` | JPEG 압축 카메라 프레임 |
| Topic | `/vision/tube_height` | `interfaces/TubeHeight` | 시약통별 높이(%) / 신뢰도 |
| Topic | `/vision/tube_state` | `interfaces/TubeState` | 시약통별 State (0/1/2/-1) |
| Topic | `/vision/yolo_result_image` | `sensor_msgs/Image` | YOLO 추론 오버레이 이미지 (HMI 표시용) |
| Topic | `/vision/hand_detected` | `std_msgs/Bool` | 작업 영역 내 손 감지 |
| Topic | `/vision/camera_status` | `std_msgs/Bool` | 카메라 프레임 수신 정상 여부 |
| Topic | `/vision/log` | `std_msgs/String` | Vision 이벤트 로그 (HMI Vision Log 탭) |
| Topic | `/robot/status` | `interfaces/RobotStatus` | 로봇 현재 상태 및 작업 로그 |
| Topic | `/robot/force_detected` | `std_msgs/Bool` | 외력 감지 여부 |
| Topic | `/robot/force_norm` | `std_msgs/Float64` | TCP Cartesian 힘 크기 (N) |
| Topic | `/robot/system_running` | `std_msgs/Bool` | 자동 루프 게이트 상태 |
| Topic | `/robot/hand_safety_enabled` | `std_msgs/Bool` | 손 감지 안전 기능 활성 상태 |
| Service | `/robot/start_task` | `interfaces/StartTask` | 상태 배열 기반 작업 시작 요청 |
| Service | `/robot/stop_task` | `interfaces/StopTask` | 비상정지 / 작업 중단 |
| Service | `/robot/set_system_running` | `std_srvs/SetBool` | HMI Start/Stop 게이트 제어 |
| Service | `/robot/set_hand_safety_enabled` | `std_srvs/SetBool` | 손 감지 안전 기능 토글 |
| Service | `/vision/request_recheck` | `interfaces/RequestRecheck` | 보충/폐기 후 재검사 요청 |
| Service | `/robot/move_to_pose` | `interfaces/MoveToPose` | named pose로 로봇 이동 |
| Service | `/gripper/control` | `interfaces/GripperControl` | 그리퍼 OPEN/CLOSE |

---

## 7. 안전 시스템

### 다층 안전 구조

| Layer | 메커니즘 | 구현 |
|---|---|---|
| 1 | **손 감지** | YOLO `hand` 클래스 N프레임 연속 검출 → 즉시 `/robot/stop_task` |
| 2 | **외력 감지** | `GetToolForce` TCP 힘 측정 `√(Fx²+Fy²+Fz²) > 20N` → `MoveStop(DR_HOLD)` |
| 3 | **카메라 watchdog** | 3초 이상 프레임 미수신 → `/vision/camera_status=False` → 작업 보류 |
| 4 | **HMI 비상정지** | 항상 최상단 노출, 즉시 동작 |

- 모든 작업 시퀀스는 `approach pose`를 경유 후 작업 위치 진입 (안전 높이 보장)
- `TaskAborted` / `TaskFailed` 예외 발생 시 항상 `home_pose`로 복귀
- 그리퍼 파지 완료는 Modbus TCP 신호로 확인 후 다음 동작 진행

상세: [docs/safety_plan.md](docs/safety_plan.md)

---

## 8. HMI 구성 (PyQt5)

| 탭 | 내용 |
|---|---|
| 메인 | 카메라 뷰 + 튜브 상태 LED + 로봇 상태 + Start/Stop/비상정지 |
| 모션 제어 | 속도·가속도 파라미터 + 손 감지 안전 토글 |
| JOG | 수동 로봇 이동 (관리자 전용) |
| 관리자 | 포즈 파라미터 설정, 시스템 파라미터 (런타임 수정, 물리 범위 검증 포함) |
| Robot Log | 로봇 작업 이벤트 로그 (중복 제거) |
| Vision Log | `/vision/log` 토픽 기반 Vision 이벤트 로그 |

---

## 9. 빠른 시작

자세한 설치/실행 방법은 **[SETUP.md](SETUP.md)** 참고.

```bash
# 1. 의존성 설치
pip3 install -r requirements.txt

# 2. 빌드
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# 3. 실행 (샘플 이미지로 zero-config 데모)
ros2 launch launch/system_launch.py

# 4. 실제 카메라 사용
ros2 launch launch/system_launch.py source_mode:=device camera_index:=0
```

`vision_pkg`에 YOLOv8n weight (`weights/reagent_yolov8n.pt`) 와 샘플 이미지 (`sample_images/`) 가 번들되어 있어 카메라 없이도 전체 파이프라인 시연 가능.

---

## 10. 폴더 구조

```
project-root/
├── README.md
├── SETUP.md                      # 환경 구성 가이드
├── requirements.txt              # pip 의존성
├── docs/
│   ├── system_architecture.md
│   ├── ros2_node_graph.md
│   ├── operation_flow.md
│   ├── safety_plan.md
│   ├── network_diagram.md
│   └── presentation_draft.md    # 발표 자료 초안
├── src/
│   ├── interfaces/              # custom msg/srv (ament_cmake)
│   ├── vision_pkg/              # camera + YOLO detector + state publisher
│   │   ├── weights/             # YOLOv8n .pt / .onnx (3-class)
│   │   ├── sample_images/       # 테스트용 샘플 이미지
│   │   └── config/              # vision_params.yaml
│   ├── robot_control_pkg/       # decision + task manager + DSR control
│   │   └── config/              # robot_params.yaml (포즈 카탈로그)
│   └── hmi_pkg/                 # PyQt5 operator dashboard
├── onnx_test_results/           # ONNX 추론 테스트 결과 이미지
└── launch/
    └── system_launch.py
```

---

## 11. 향후 개선 사항

- 비전 기반 포즈 자동 캘리브레이션 (현재 수동 티칭)
- 조명 변화 대응 (HSV 정규화)
- 웹 기반 원격 모니터링 대시보드
- 시약 종류 분류 클래스 추가 (라벨 확장)
- 3D 공간 occupancy 기반 충돌 예측

---

## 12. 팀 구성

| 이름 | 역할 |
|---|---|
| jiwan | 시스템 통합, 외력 감지, 로봇 제어 |
| | Vision AI, YOLO 학습, 추론 파이프라인 |
| | HMI (PyQt5), 관리자 탭, 파라미터 관리 |
| | 로봇 모션 시퀀스, 포즈 카탈로그 |

전체 동작 순서: [docs/operation_flow.md](docs/operation_flow.md)
