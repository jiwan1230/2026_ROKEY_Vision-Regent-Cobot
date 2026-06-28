# ROS2 Node Graph

## 노드 연결 개요

```
[Vision Sub PC]
  side_camera_node
      │ /vision/side_image (CompressedImage, BEST_EFFORT)
      │ /vision/log (String)
      ▼
  liquid_height_detector_node
      │ /vision/tube_height (TubeHeight)
      │ /vision/hand_detected (Bool)
      │ /vision/camera_status (Bool)
      │ /vision/yolo_image (Image)  ← YOLO 추론 ON 시
      │ /vision/log (String)
      ▼
  tube_state_publisher_node
      │ /vision/tube_state (TubeState)
      │ /vision/log (String)
      │
      │ [Services 호스팅]
      │  /vision/request_recheck
      │  /vision/mark_tube_disposed
      │  /vision/reset_handled_slots
      │  /robot/current_tube_state
      │
[Main PC]
      ▼
  main_decision_node
      │ /robot/system_running (Bool) ──────────────────────────► hmi_node
      │ /robot/hand_safety_enabled (Bool) ────────────────────► hmi_node
      │
      │ [Services 호스팅]
      │  /robot/set_system_running (SetBool)
      │  /robot/set_hand_safety_enabled (SetBool)
      │
      ▼ /robot/start_task (StartTask)
  robot_task_manager_node
      │ /robot/status (RobotStatus) ──────────────────────────► hmi_node
      │ /robot/tray_advanced (Int32) ─────────────────────────► main_decision_node
      │
      │ [Services 호스팅]
      │  /robot/start_task (StartTask)
      │  /robot/stop_task (StopTask)
      │
      ├──► /robot/move_to_pose ──────────────────────────────────────┐
      │                                                               ▼
      │                                               doosan_robot_control_node
      │                                                   │  Doosan DSR API
      │                                                   │  movel / movej
      │                                                   │  GetToolForce
      │                                                   │
      │                                                   │ /robot/force_detected (Bool) ► hmi_node
      │                                                   │ /robot/force_norm (Float64) ► hmi_node
      │                                                   │
      │                                                   │ [Services 호스팅]
      │                                                   │  /robot/move_to_pose (MoveToPose)
      │                                                   │  /robot/add_tcp (AddTcp)
      │                                                   │  /robot/set_tcp (SetTcp)
      │                                                   │  /robot/hard_stop (Trigger)
      │
      └──► /gripper/control ─────────────────────────────────────────┐
                                                                      ▼
                                                         gripper_control_node
                                                             │  Modbus TCP (192.168.1.1:502)
                                                             │  Register 268 폴링
                                                             │  Doosan digital I/O
                                                             │
                                                             │ [Services 호스팅]
                                                             │  /gripper/control (GripperControl)
```

## Node 역할

| Node | 패키지 | 역할 |
|------|--------|------|
| `side_camera_node` | vision_pkg | USB 카메라(또는 테스트 이미지/영상) → `CompressedImage` 발행 (BEST_EFFORT QoS). `/camera/resolution_cmd`로 해상도 런타임 변경 가능 |
| `liquid_height_detector_node` | vision_pkg | YOLOv8n(cup/height/hand) 추론. cup bbox 3구역 매핑 → height bbox 비율로 액체 높이 산출. hand ROI y-min 필터(`hand_roi_y_min_px=250`) 적용 |
| `tube_state_publisher_node` | vision_pkg | 임계값 로직으로 State 0/1/2/UNKNOWN 분류. 폐기 완료 처리(`mark_tube_disposed`), 슬롯 앵커 리셋 서비스 호스팅 |
| `main_decision_node` | robot_control_pkg | tube_state + 안전 토픽 구독. hand/force/camera 게이트 통과 시 `/robot/start_task` 호출. busy-flag, tray_transferred 플래그로 중복 dispatch 방지 |
| `robot_task_manager_node` | robot_control_pkg | StartTask 수신 → 우선순위(State2 > State0 > State1) 1건 처리 후 recheck. 트레이 이송(3개 트레이) 포함. `/robot/tray_advanced` 발행으로 트레이 인덱스 변경 알림 |
| `doosan_robot_control_node` | robot_control_pkg | Doosan DSR API(`movel`/`movej`)로 실제 로봇 제어. `GetToolForce`로 TCP 외력 측정(20N 임계값). Virtual TCP(`apply_virtual_tcp`) 적용. tcp_offset ROS 파라미터로 런타임 조정 가능 |
| `gripper_control_node` | robot_control_pkg | OnRobot RG2 제어. Doosan `set_digital_output`으로 OPEN/CLOSE 트리거, Modbus TCP(Register 268)로 동작 완료 폴링 |
| `hmi_node` | hmi_pkg | PyQt5 GUI. 모든 상태 토픽 구독. 운영 대시보드(카메라/튜브 상태/로봇 상태/안전 패널) + 시스템 관리자 탭(파라미터/TCP/포즈 이동) + Force Monitor 탭(실시간 외력 그래프) |

## Topic 정의

| Topic | 타입 | 발행 노드 | 설명 |
|-------|------|-----------|------|
| `/vision/side_image` | `CompressedImage` | side_camera_node | 카메라 원본 프레임 (BEST_EFFORT) |
| `/vision/yolo_image` | `Image` | liquid_height_detector_node | YOLO 추론 결과 시각화 |
| `/vision/tube_height` | `interfaces/TubeHeight` | liquid_height_detector_node | `tube_index[]`, `liquid_height[]`(mm), `confidence[]` |
| `/vision/tube_state` | `interfaces/TubeState` | tube_state_publisher_node | `tube_index[]`, `state[]` (0/1/2/-1) |
| `/vision/hand_detected` | `Bool` | liquid_height_detector_node | 작업영역 내 손 감지 (y > 250px ROI 필터 적용) |
| `/vision/camera_status` | `Bool` | liquid_height_detector_node | 카메라 프레임 정상 수신 여부 |
| `/vision/log` | `String` | side_camera_node, liquid_height_detector_node, tube_state_publisher_node | Vision 이벤트 로그 |
| `/robot/status` | `interfaces/RobotStatus` | robot_task_manager_node | `status`, `current_task`, `detail`, `log` |
| `/robot/force_detected` | `Bool` | doosan_robot_control_node | TCP 외력 20N 초과 여부 |
| `/robot/force_norm` | `Float64` | doosan_robot_control_node | TCP 외력 크기 (N), 실시간 |
| `/robot/force_detected` | `Bool` | doosan_robot_control_node | 외력 임계 초과 여부 |
| `/robot/tray_advanced` | `Int32` | robot_task_manager_node | 트레이 인덱스 변경 알림 |
| `/robot/system_running` | `Bool` | main_decision_node | 자동 dispatch 게이트 상태 |
| `/robot/hand_safety_enabled` | `Bool` | main_decision_node | 손 감지 자동정지 활성 여부 |
| `/vision/yolo_enabled` | `Bool` | hmi_node | YOLO 추론 on/off 런타임 토글 |
| `/camera/resolution_cmd` | `String` | hmi_node | 카메라 해상도 변경 명령 |

## Service 정의

| Service | 타입 | 호스트 노드 |
|---------|------|-------------|
| `/robot/start_task` | `interfaces/StartTask` | robot_task_manager_node |
| `/robot/stop_task` | `interfaces/StopTask` | robot_task_manager_node |
| `/robot/move_to_pose` | `interfaces/MoveToPose` | doosan_robot_control_node |
| `/robot/add_tcp` | `interfaces/AddTcp` | doosan_robot_control_node |
| `/robot/set_tcp` | `interfaces/SetTcp` | doosan_robot_control_node |
| `/robot/hard_stop` | `Trigger` | doosan_robot_control_node |
| `/robot/current_tube_state` | `interfaces/CurrentTubeState` | tube_state_publisher_node |
| `/robot/set_system_running` | `SetBool` | main_decision_node |
| `/robot/set_hand_safety_enabled` | `SetBool` | main_decision_node |
| `/gripper/control` | `interfaces/GripperControl` | gripper_control_node |
| `/vision/request_recheck` | `interfaces/RequestRecheck` | tube_state_publisher_node |
| `/vision/mark_tube_disposed` | `interfaces/MarkTubeDisposed` | tube_state_publisher_node |
| `/vision/reset_handled_slots` | `Trigger` | tube_state_publisher_node |
| `/vision/reset_slot_anchors` | `Trigger` | liquid_height_detector_node |
| `/vision/reset_slot_anchors` | `Trigger` | liquid_height_detector_node |

## ROS2 런타임 파라미터 (주요)

| 노드 | 파라미터 | 기본값 | 설명 |
|------|----------|--------|------|
| doosan_robot_control_node | `force_threshold` | 20.0 | 외력 감지 임계값 (N) |
| doosan_robot_control_node | `tcp_offset` | `[0,0,200,0,0,0]` | Virtual TCP 오프셋 (런타임 변경 가능) |
| doosan_robot_control_node | `velocity` / `acceleration` | 60 / 60 | 로봇 속도/가속도 (%) |
| liquid_height_detector_node | `hand_roi_y_min_px` | 250.0 | 손 감지 ROI 하단 y 기준 |
| liquid_height_detector_node | `confidence_threshold` | 0.5 | YOLO 검출 confidence 하한 |
