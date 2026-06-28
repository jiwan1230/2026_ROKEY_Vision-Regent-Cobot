# System Architecture

## 전체 구성

```
┌──────────────────────────────────────────────────────────────────┐
│                    Vision AI Reagent QC 시스템                    │
│                                                                  │
│  ┌─────────────────┐     ROS2 DDS      ┌──────────────────────┐ │
│  │  Vision Sub PC  │ ◄───────────────► │      Main PC         │ │
│  │                 │                   │                      │ │
│  │ side_camera     │                   │ main_decision_node   │ │
│  │ YOLOv8n 추론    │                   │ robot_task_manager   │ │
│  │ 상태 분류       │                   │ doosan_control       │ │
│  │                 │                   │ gripper_control      │ │
│  └────────┬────────┘                   │ hmi_node (PyQt5)     │ │
│           │                            └──────────┬───────────┘ │
│           │ USB                                   │             │
│  Side-view Camera                                 │ Doosan SDK  │
│                                         ┌─────────┴──────────┐  │
│                                         │   Doosan M0609     │  │
│                                         │   협동로봇 암       │  │
│                                         └─────────┬──────────┘  │
│                                                   │             │
│                                     Modbus TCP / digital I/O    │
│                                         ┌─────────┴──────────┐  │
│                                         │  OnRobot RG2       │  │
│                                         │  그리퍼             │  │
│                                         └────────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

## 구성 요소

| 구성 요소 | 역할 | 구현 |
|-----------|------|------|
| Side-view Camera | 시약통 3개 측면 영상 취득 | USB 카메라, CompressedImage 발행 |
| Vision Sub PC | YOLOv8n 실시간 추론, 상태 분류 | `vision_pkg` (ROS2) |
| Main PC | 판단 로직, 로봇 명령, HMI | `robot_control_pkg`, `hmi_pkg` (ROS2) |
| Doosan M0609 | 보충/폐기/트레이 이송 실행 | Doosan DSR API (`movel`, `movej`) |
| OnRobot RG2 | 시약통/트레이 파지 | Modbus TCP + Doosan digital I/O |

## 데이터 흐름

```
Side-view Camera
  → CompressedImage (/vision/side_image)
  → YOLOv8n 추론 (cup / height / hand 클래스)
  → 액체 높이 산출 (fill fraction × calibration)
  → 상태 분류 (State 0: 부족 / 1: 정상 / 2: 과다 / -1: UNKNOWN)
  → /vision/tube_state
  → main_decision_node (안전 게이트: camera_ok / hand / force)
  → /robot/start_task
  → robot_task_manager_node (우선순위: 폐기 > 보충 > 이송)
  → /robot/move_to_pose + /gripper/control
  → M0609 + RG2 (실제 동작 수행)
  → /vision/request_recheck
  → 다음 사이클
```

## 안전 피드백 루프

```
M0609 TCP 외력 측정 (GetToolForce, ~7Hz)
  → force_norm (Float64) → HMI Force Monitor 탭 실시간 그래프
  → force_norm > 20N
    → /robot/force_detected = True
    → 진행 중 동작 즉시 정지 (stop_event)
    → HMI Force Alert 팝업
    → 작업자 확인 → Start 버튼 → 재개
```

## 핵심 설계 원칙

1. **반응형 사이클**: 매 `/vision/tube_state` 수신마다 1건 처리 후 종료. Vision 파이프라인은 항상 병렬로 동작하므로 별도 recheck 루프 없이 자동 진행됨.

2. **우선순위 처리**: 동일 사이클에 폐기·보충이 동시 존재해도 항상 폐기(State2) → 보충(State0) → 이송(All 1) 순서 보장.

3. **다중 안전 게이트**: 카메라 단절, 손 감지, 외력 초과 각각 독립적으로 dispatch를 차단. 외력은 Start 버튼으로만 해제(자동 재개 없음).

4. **트레이 3개 순차 이송**: tray_idx 0→1→2 순서. 충돌 회피 경유지(TRAY_TOOL_STAND_APPROACH_POSE, tool_approach_pose(0)) 포함.

5. **런타임 파라미터 조정**: TCP 오프셋, 속도/가속도, 외력 임계값, Vision confidence 등을 HMI 시스템 관리자 탭에서 재시작 없이 변경 가능.
