# 안전 관리 설계

## 안전 계층 구조

```
레벨 1 (하드웨어): 로봇 제한 속도, 작업영역 물리적 펜스
레벨 2 (외력 감지): GetToolForce 폴링 → 20N 초과 시 즉시 정지
레벨 3 (Vision 안전): 손 감지 ROI 필터 → 작업 중단/재개
레벨 4 (시스템 게이트): camera_ok / force_detected / hand_detected 복합 조건
레벨 5 (HMI): Emergency Stop 버튼, Force Alert 팝업, 수동 Start/Stop
```

## 구현된 안전 기능

| 안전 대책 | 구현 방법 | 관련 코드 |
|-----------|----------|-----------|
| **외력 감지 즉시 정지** | `GetToolForce` (~7Hz 폴링) → force_norm > 20N 시 stop_event 세팅 | `doosan_robot_control_node.py` |
| **외력 실시간 모니터링** | `/robot/force_norm` (Float64) → HMI Force Monitor 탭 그래프 | `hmi_node.py` |
| **손 감지 작업 중단** | YOLOv8n hand 클래스 + y>250px ROI 필터 → 동작 일시 정지, 손 사라지면 자동 재개 | `liquid_height_detector_node.py` |
| **카메라 단절 보호** | watchdog 타임아웃 → `camera_status=False` → dispatch 차단 | `liquid_height_detector_node.py` |
| **HMI Emergency Stop** | 버튼 → `/robot/stop_task(stop=True, is_emergency=True)` | `hmi_node.py` |
| **approach 경유 강제** | 모든 pick/refill 동작은 approach_pose 경유 후 진입 | `robot_task_manager_node.py` |
| **그리퍼 파지 재시도** | `grip_with_retry()` → 실패 시 1회 재시도 후 ERROR | `robot_task_manager_node.py` |
| **UNKNOWN 상태 보류** | STATE_UNKNOWN(-1) 존재 시 start_task 호출 금지 | `main_decision_node.py` |
| **TCP 변경 잠금** | `system_running=True` 또는 로봇 비IDLE 상태에서 tcp_offset 변경 불가 | `doosan_robot_control_node.py` |
| **속도/가속도 제한** | `velocity=60, acceleration=60` (기본값), HMI에서 런타임 조정 가능 | `robot_params.yaml` |
| **트레이 충돌 회피** | tray 이송 시 경유 웨이포인트로 인접 트레이 컵과의 충돌 방지 | `robot_task_manager_node.py` |
| **외력 해제는 수동** | 외력 감지 후 Start 버튼으로만 재개 (자동 재개 없음) | `main_decision_node.py` |

## 외력 감지 상세 동작

```
doosan_robot_control_node (약 7Hz 타이머):
  GetToolForce() → [Fx, Fy, Fz, Tx, Ty, Tz]
  force_norm = sqrt(Fx² + Fy² + Fz²)
  
  force_norm > 20N (robot_params.yaml: force_threshold)
    → /robot/force_detected = True 발행
    → main_decision_node: force_detected=True, dispatch 차단
    → robot_task_manager_node: stop_event 세팅, 진행 중 movel 중단
    → hmi_node: Force Alert 팝업 래치 (Start 전까지 유지)
  
  /robot/force_norm 발행 (항상, 실시간 모니터링용)
    → HMI Force Monitor 탭: 30초 롤링 그래프 갱신
```

## 손 감지 상세 동작

```
liquid_height_detector_node:
  YOLOv8n 'hand' 클래스 검출
  → y 좌표 중심 >= 250px (hand_roi_y_min_px) 인 경우만 유효
    (로봇 암 자체가 상단에서 hand로 오검출되는 것 필터링)
  → /vision/hand_detected = True 발행

main_decision_node:
  hand_detected=True + busy=True + hand_safety_enabled=True
    → /robot/stop_task(stop=True) 호출 (비상정지 아님, 일시 정지)
  
  hand_detected=False (손 사라짐) + busy=True
    → /robot/stop_task(stop=False) 호출 → 동작 재개
```

## 오류 처리 시나리오

### Vision 오류

| 오류 상황 | 처리 방법 |
|-----------|----------|
| 카메라 연결 실패 | watchdog 타임아웃 → camera_status=False → dispatch 보류, HMI DISCONNECTED 표시 |
| 시약통 미검출 | cup/height 매칭 실패 → confidence=0 → STATE_UNKNOWN(-1) → dispatch 보류 |
| confidence 낮음 | confidence_threshold 미달 → STATE_UNKNOWN → HMI 회색 표시 |
| 높이 이상값 | target±tolerance 밖 → State 2(폐기) 자동 분류 |

### Robot 오류

| 오류 상황 | 처리 방법 |
|-----------|----------|
| 로봇 이동 실패 | `move()` 응답 실패 → TaskFailed → STATUS_ERROR → HMI 표시 |
| 그리퍼 파지 실패 | `grip_with_retry()` 1회 재시도 → 실패 시 TaskFailed |
| 외력 감지 | 즉시 정지 → HMI Alert → 작업자 Start 버튼으로 재개 |
| TCP 외력 측정 실패 | GetToolForce 예외 → 로그 경고만 (측정 스킵, 동작 계속) |

### Communication 오류

| 오류 상황 | 처리 방법 |
|-----------|----------|
| Vision PC 통신 끊김 | camera_status watchdog → False → 동작 보류 |
| Modbus TCP 연결 실패 | gripper_control_node 시작 시 재연결 시도, 실패 시 로그 경고 |
| ROS2 서비스 미응답 | `_call_sync()` 타임아웃 → TaskFailed |
| 상태 배열 불완전 | `len(tube_index) < num_tubes` → dispatch 보류 |

## 비상정지 복구 절차

1. 로봇 완전 정지 확인
2. 위험 요인 제거 (장애물, 이상 외력 원인)
3. HMI Force Alert 팝업 확인
4. HMI **Start** 버튼 클릭 → force_detected 리셋, 시스템 재개
5. 필요 시 `scripts/restart_system.sh` 실행으로 전체 재시작
