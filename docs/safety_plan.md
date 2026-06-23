# 안전 관리 설계

본 시스템은 협동로봇과 시약을 함께 다루므로 안전 설계가 필수이다.

## 기본 안전 대책 (구현 매핑)

| 대책 | 구현 |
|---|---|
| 로봇 작업 영역 명확히 구분 | `config/robot_params.yaml`의 named pose가 approach/pick/refill로 분리되어 영역이 고정됨 |
| 비상정지 버튼을 작업자가 즉시 누를 수 있는 위치에 둠 | HMI Control Panel의 `EMERGENCY STOP` 버튼 → `/robot/stop_task` |
| 로봇 동작 속도 제한 | `doosan_robot_control_node`의 `velocity`/`acceleration` 파라미터 (실제 연동 시 `movel(vel=, acc=)`로 전달) |
| 파지 및 접근 동작은 저속으로 수행 | approach pose를 경유한 뒤에만 pick/refill pose로 진입 (모든 작업 시퀀스가 이 순서를 따름) |
| 시약통 파지 전후 그리퍼 상태 확인 | `grip_with_retry()`가 실패 시 1회 재시도 후 작업을 ERROR로 종료 |
| 카메라 인식 실패 시 로봇이 동작하지 않도록 함 | `/vision/camera_status=False` → `main_decision_node`가 `/robot/start_task` 호출을 보류 |
| 시약 보충/폐기 동작은 반드시 사전 정의된 좌표에서만 수행 | `doosan_robot_control_node`가 `config/robot_params.yaml`에 등록된 pose_name만 허용 (미등록 이름은 실패 응답) |
| 로봇 동작 중 작업자가 접근하면 작업을 중단할 수 있도록 함 | `hand` 클래스 검출 → `/vision/hand_detected=True` → `main_decision_node`가 즉시 `/robot/stop_task` 호출 |
| 모든 오류는 HMI와 로그에 기록 | `/robot/status`(ERROR/EMERGENCY_STOP) + HMI Log Panel + Safety Panel |
| 실제 위험 시약 대신 안전한 대체 액체로 먼저 검증 | (운영 절차 - 코드 범위 밖) |

## 오류 처리 시나리오

### Vision 오류

| 오류 상황 | 처리 방법 | 구현 |
|---|---|---|
| 카메라 연결 실패 | HMI에 오류 표시, 로봇 동작 금지 | `camera_timeout_sec` 초과 시 `/vision/camera_status=False`; `main_decision_node`가 동작 보류 |
| 시약통 검출 실패 | 해당 index를 UNKNOWN으로 표시, 수동 확인 요청 | cup/height 매칭 실패 시 confidence=0.0 → `tube_state_publisher_node`가 STATE_UNKNOWN(-1) 분류 |
| confidence 낮음 | 재촬영 또는 조명 확인 요청 | `confidence_threshold` 미달 시 STATE_UNKNOWN, HMI에 회색으로 표시 |
| 높이 추론값 이상 | State 2 또는 Manual Check로 처리 | 임계값 로직상 `target±tolerance` 밖이면 자동으로 State 2(폐기) 분류됨 |

### Robot 오류

| 오류 상황 | 처리 방법 | 구현 |
|---|---|---|
| 로봇 이동 실패 | 즉시 정지 후 HMI 오류 표시 | `move()`가 `response.success=False`면 `TaskFailed` 발생 → `/robot/status=ERROR` |
| 그리퍼 파지 실패 | 재시도 1회 후 실패 시 수동 확인 | `grip_with_retry()` (`grip_retry_count` 파라미터) |
| 외력 감지 이상 | 로봇 정지 및 작업자 확인 요청 | (실제 하드웨어 연동 시 `DSR_ROBOT2`의 외력 감지 콜백을 `/robot/stop_task` 호출에 연결 - 현재 mock에는 외력 센서 없음) |
| 좌표 접근 실패 | 해당 작업 중단 후 home pose 복귀 | `TaskFailed`/`TaskAborted` 발생 시 현재 시퀀스를 즉시 중단 (모든 시퀀스가 정상 종료 시 `home_pose`로 복귀하도록 설계됨) |

### Communication 오류

| 오류 상황 | 처리 방법 | 구현 |
|---|---|---|
| Vision PC와 Main PC 통신 끊김 | 로봇 동작 중지 | `/vision/camera_status` 갱신 중단 시 watchdog이 `False`로 보고 → 동작 보류 |
| ROS2 Topic 수신 지연 | 최신 데이터만 사용, 오래된 데이터 폐기 | 각 구독자가 콜백마다 `latest_*` 캐시를 덮어씀 (오래된 메시지 큐잉 없음, QoS depth=10) |
| 상태 배열 누락 | 작업 시작 금지 | `main_decision_node`가 `len(msg.tube_index) < num_tubes`면 작업 보류 |
| 재검사 응답 없음 | Timeout 처리 후 수동 확인 | `/vision/request_recheck`가 `recheck_wait_timeout_sec` 내 새 상태가 없으면 `success=False` 반환; `robot_task_manager_node`는 이를 로그로 경고만 남기고 작업 자체는 완료 처리 (다음 사이클이 자연 복구) |
