# 주요 버그 해결 사례

> git commit 기록 기반 실제 발생 버그 및 해결 과정 정리.

---

## 1. Vision — 시약통 인덱스 밀림 (트레이 교체 시 인식 오류)

**커밋:** `16b603b` (2026-06-24)

### 증상
트레이를 교체하거나 폐기로 컵 1개가 사라지면, 남은 컵들의 tube_index가
한 칸씩 밀려 0번 자리에 있던 컵이 1번으로 인식되는 오류 발생.
실제로는 정상인 컵에 잘못된 보충 명령이 내려지거나, 이미 폐기한 컵 위치로
로봇이 이동하는 사고가 발생.

### 원인
기존 `assign_tube_order()`가 매 프레임마다 검출된 cup bbox를 x 좌표 기준으로
재정렬(re-rank)했기 때문. 컵이 3개일 때는 맞지만, 폐기로 1개가 사라지면
남은 2개가 index 0, 1로 재배치되어 원래 위치 정보가 사라짐.

### 해결
**동적 슬롯 앵커(slot anchor)** 방식으로 전면 교체.
- 처음 num_tubes개의 컵이 동시에 검출되는 순간 `(cx, y1)` 좌표를 앵커로 고정
- 이후 프레임에서는 고정된 앵커와 x/y 허용 오차 내에서 매칭
- 컵이 사라진 슬롯은 `None`으로 처리, 해당 슬롯의 height 버퍼를 클리어
- 트레이 교체 시 `/vision/reset_slot_anchors` 서비스로 앵커 리셋

---

## 2. Vision — cup/height bbox 위치 유사로 슬롯 매칭 오류

**커밋:** `2f67473` (2026-06-29)

### 증상
파인튜닝 모델 교체 후, 시약통 인식은 되지만 특정 슬롯에 엉뚱한 높이값이
매핑되거나 tube_index가 잘못 배정되는 현상. 특히 가운데 컵에서 자주 발생.

### 원인
`match_cups_to_anchors()`의 score 공식이 `box.conf - dx*0.01 - dy*0.01`이어서
**confidence 값이 위치 오차보다 압도적으로 강하게 작용**.
cup bbox와 height bbox가 비슷한 위치에 존재할 때, 신뢰도 높은 height bbox가
cup 슬롯을 가로채는 상황이 발생.

### 해결
score 공식을 `-(dx + dy)` (순수 위치 기반)으로 변경.
앵커에 공간적으로 가장 가까운 박스를 선택하도록 수정.
confidence는 슬롯 매핑 순서에 영향을 주지 않음.

```python
# Before
score = box.conf - dx * 0.01 - dy * 0.01

# After
score = -(dx + dy)
```

---

## 3. 카메라 — 영상 전송 FPS 1~3Hz로 저하

**커밋:** `3535920` (2026-06-24)

### 증상
Vision 파이프라인 목표 7Hz를 달성하지 못하고 1~3Hz로 떨어지며,
최대 2.4초 간격으로 프레임이 끊기는 현상. HMI 영상이 불규칙하게 버벅임.

### 원인
`/vision/side_image`를 raw `bgr8` 포맷으로 발행 시 한 프레임 크기가 **921 KB**.
BEST_EFFORT QoS의 UDP 전송에서 패킷 단편화 발생 → 단편 하나라도 손실되면
재전송 없이 **프레임 전체 드롭**.

### 해결
`sensor_msgs/Image` → **`sensor_msgs/CompressedImage` (JPEG)** 로 전환.
JPEG 압축으로 페이로드를 약 1/10로 절감, 단편화 확률 대폭 감소.
QoS도 publisher/subscriber 양쪽 BEST_EFFORT로 통일.
변경 후 측정: 7.00 Hz, 표준편차 < 2ms로 안정화.

---

## 4. 비상 정지 — 동작 중 즉시 정지 안 됨

**커밋:** `722530e`, `ebf89d5`, `c873365` (2026-06-25~26)

### 증상
HMI Emergency Stop 버튼을 눌러도 로봇이 현재 `movel()` 완료 후
**다음 체크포인트에서야** 멈추는 문제. 길거나 느린 이동 중에는 수 초간 정지가
지연됨.

### 원인 (3단계로 진화한 버그)

**1차:** `stop_event`를 `_call_sync` 루프 끝에서만 확인해 movel()이 끝날 때까지 대기.

**2차 수정 후 새 문제:** `spin_until_future_complete()`를 executor 내부에서
다시 호출하는 중첩 실행 → deadlock 발생, `_call_sync` 영구 행(hang).

**3차:** `stop_event` 세팅 시 `future.cancel()`로 즉시 서비스 요청을 취소했으나,
DSR API는 cancel이 이미 진행 중인 `movel()`을 중단하지 않음.

### 해결
`move_stop` 서비스(`motion/move_stop`)를 **별도 dsr_stop_node**에서 호출하도록 분리.
`stop_event` 세팅과 동시에 `move_stop`을 비동기로 호출해 하드웨어 레벨에서 즉시 중단.
`_call_sync`는 `stop_event` 확인 즉시 `TaskAborted` 발생으로 제어 반환.

---

## 5. 외력 감지 — 관절 토크로는 자세별 편차가 너무 큼

**커밋:** `243bbbc`, `898fea4` (2026-06-27)

### 증상
외력 감지 기준값(`force_threshold`)을 아무리 튜닝해도 로봇 자세가 바뀔 때마다
오감지가 발생. 시약을 파지하거나 특정 각도로 이동 시 임계값 초과.
반대로 너무 높이 설정하면 실제 충돌에도 반응 안 함.

### 원인
`GetExternalTorque()`(관절 토크 벡터)를 사용 → 자세마다 중력/관성이 다르게
반영되어 **같은 외력이 자세에 따라 토크로 환산되는 값이 달라짐**.
또한 비동기 요청 없이 `spin_until_future_complete()`를 타이머 콜백 내에서 호출해
movel() 블락 중 외력 체크가 실행되지 않음 (최대 1초 공백).

### 해결
`GetExternalTorque` → **`GetToolForce`(TCP 카르테시안 힘)**로 전환.
- 페이로드(payload) 설정으로 시약통 무게를 자동 보상 → 자세별 변동 제거
- `√(Fx² + Fy² + Fz²)` 벡터 크기로 방향 무관 감지
- `ReentrantCallbackGroup`으로 타이머와 movel() 콜백을 독립 실행
- `_force_pending` 가드 제거 → 매 틱마다 무조건 요청 발행
- 임계값 20N으로 최종 확정

---

## 6. 손 감지 — grip/approach 동작 중 작업 중단

**커밋:** `f72a1d4` (2026-06-26)

### 증상
작업자가 손을 카메라에 보이면 grip/approach 이동 중에도 `TaskAborted`가 발생해
시약통을 집다 말고 중간에 작업이 종료됨. 이후 재시작 시 로봇 상태 불일치 발생.

### 원인
`move()`에서 `stop_event` 감지 시 항상 `TaskAborted`를 발생시키도록 구현되어 있어,
손 감지(일시 정지 의도)와 비상 정지(종료 의도)를 구분하지 못했음.

### 해결
`move()`에 `abort_on_stop` 파라미터 추가:
- `abort_on_stop=False` (기본값, grip/approach): 손 감지 시 정지 후 대기, 손이
  사라지면 **자동 재시도**. 비상 정지만 `TaskAborted` 발생.
- `abort_on_stop=True` (pour rotate 전용): `TaskAborted`를 호출부(pour 루프)로
  전달해 시약통 직립 처리를 루프가 담당.

---

## 7. 그리퍼 — 공중에서 그리퍼가 열려 시약통 낙하

**커밋:** `a97f8b9` (2026-06-26)

### 증상
비상 정지 또는 손 감지로 cleanup 동작이 실행될 때, 시약통을 집으러 가는 도중
아직 grip_pose에 도달하지 않은 상태에서 `gripper OPEN` 명령이 실행되어
**시약통이 공중에서 낙하**하는 심각한 사고 발생.

### 원인
cleanup을 `ignore_stop=True`로 실행하면, `hard_stop`이 이동을 중단시켜도
서비스가 `success=True`를 반환하면 즉시 다음 단계(gripper OPEN)로 진행했음.
실제로는 grip_pose에 미도달한 상태.

### 해결
`_do_cleanup(fn)` 헬퍼 도입:
- 각 cleanup 단계(approach → grip_pose → open → home)가 **실제 완료된 후**
  다음 단계 실행을 보장
- `TaskAborted` 발생 시 `stop_event` 해제를 대기한 후 해당 단계를 **재시도**
- 손 감지 시: move() 내부 흡수(pause) → 재시도
- 비상 정지 시: Start 버튼까지 대기 후 재시도

---

## 8. tray_transferred 플래그 — 2번째 트레이 이송 안 됨

**커밋:** `5b6952c`, `422f869` (2026-06-28)

### 증상
0번 트레이 이송 성공 후 트레이가 1번으로 전환되어도, 1번 트레이 상태가
all_normal임에도 불구하고 이송이 실행되지 않고 시스템이 정지.

### 원인 (2단계)

**1차 원인:** `_advance_tray()` 후 `tray_transferred=True`가 유지됨.
`main_decision_node`는 트레이 인덱스가 바뀐 것을 알 방법이 없어, 새 트레이도
"이미 이송 완료"로 간주해 dispatch 차단.

**2차 원인(레이스 컨디션):** `/robot/tray_advanced` 토픽 발행으로 1차 수정 후에도,
`on_tray_advanced`(tray_transferred=False)가 `on_start_task_done`
(tray_transferred=True)에 **3ms 차이로 덮어써지는** 레이스 컨디션 발생.

### 해결
`_tray_advanced_during_task` 플래그 추가:
- `on_tray_advanced`: 플래그를 `True`로 기록
- `on_start_task_done`: 플래그가 `True`이면 `tray_transferred=True`를 세우지 않음
- 태스크 완료마다 플래그 리셋

---

## 9. HMI — Qt 스레드 외 접근으로 크래시

**커밋:** `c707993` (2026-06-25)

### 증상
Start/Stop 등 버튼 클릭 직후 HMI가 비정상 종료.
`"Cannot queue arguments of type QTextCursor"` 에러 발생.

### 원인
`future.add_done_callback()`은 rclpy spin 스레드에서 실행됨.
콜백 내에서 `self.log()`를 직접 호출하면 **GUI 스레드가 아닌 곳에서
Qt 위젯(`textEdit`)을 건드리게 되어** Qt 내부 규칙 위반으로 크래시.

### 해결
`pyqtSignal(str)`로 문자열만 전달 후 Qt가 GUI 스레드로 마샬링하도록 변경.
이후 모든 ROS 콜백 → Qt 위젯 업데이트 경로를 `pyqtSignal` 패턴으로 통일.

---

## 10. 시스템 — 재시작 시 set_singularity_handling 타임아웃

**커밋:** `57b77f2` (2026-06-28)

### 증상
`restart_system.sh` 실행 후 재시작하면 `set_singularity_handling` 서비스 호출이
타임아웃되고 `doosan_robot_control_node`가 초기화에 실패.

### 원인
`ros2_control_node`와 `dsr_controller`가 SIGINT로 종료되지 않고 **좀비 프로세스로
잔존**, DDS 엔드포인트를 점유한 상태에서 새 노드가 동일 엔드포인트를 사용하려 해
통신이 꼬임.

### 해결
`restart_system.sh`의 kill 대상 목록에 `ros2_control_node`, `dsr_controller` 추가.
SIGINT 후 3초 대기, 미종료 시 SIGTERM으로 이중 보장.

---

## 11. 보충 — refill 완료 판단 오류로 조기 종료

**커밋:** `8218425` (2026-06-26)

### 증상
시약 보충 중 시약통이 완전히 차지 않았는데 `NORMAL` 판정이 내려져 보충이
조기 종료되는 현상. Vision 노이즈 1프레임으로 State 1 오판정.

### 원인
단일 프레임에서 `STATE_NORMAL` 응답이 오면 즉시 보충 루프를 종료하도록 구현.
조명 변화나 시약 출렁임으로 인한 1프레임 노이즈에 취약.

### 해결
N회 연속 `STATE_NORMAL`이 확인될 때만 보충 완료로 인정.
```python
# consecutive_normal_count >= NORMAL_CONFIRM_COUNT 일 때만 종료
```
`normal_confirm_count` 파라미터로 런타임 조정 가능.

---

## 12. 외력 감지 HMI — 팝업 깜빡임 및 Start 후 재표시

**커밋:** `4cf04a7` (2026-06-27)

### 증상
- 외력이 순간적으로 임계값을 넘었다가 바로 사라져도 팝업이 표시됨
- Start 버튼 클릭 후에도 외력 센서가 아직 True이면 팝업이 **즉시 재표시**
- `_refresh_dashboard()` 150ms 루프에서 조건 변화마다 팝업이 깜빡임

### 해결
**상승 에지(False → True) 래치** 방식으로 변경:
- `_force_latch`: False → True 전환 순간 한 번만 래치를 세움
- `_force_shown`: 팝업이 현재 표시 중인지 상태 추적
- 래치는 HMI Start 버튼 클릭 시에만 해제 (순간적 외력 소멸로 자동 해제 안 됨)
- Start 클릭 후 `_force_acknowledged` 플래그로 센서 값이 아직 True여도 재표시 방지

---

## 요약 테이블

| # | 버그 분류 | 핵심 원인 | 해결 방식 |
|---|-----------|----------|----------|
| 1 | Vision 인덱스 밀림 | 매 프레임 재정렬 | 슬롯 앵커 고정 |
| 2 | 슬롯 매칭 오류 | confidence 우선 score | 위치 기반 score |
| 3 | 카메라 FPS 저하 | raw 이미지 UDP 단편화 | JPEG CompressedImage |
| 4 | 비상 정지 지연 | 체크포인트 방식 정지 | move_stop 하드웨어 즉시 중단 |
| 5 | 외력 감지 오작동 | 관절 토크 자세 편차 | TCP Cartesian force |
| 6 | 손 감지 → 작업 중단 | stop 시 항상 TaskAborted | abort_on_stop 파라미터화 |
| 7 | 그리퍼 공중 낙하 | ignore_stop 무조건 진행 | _do_cleanup 헬퍼 재시도 보장 |
| 8 | 트레이 이송 미수행 | tray_transferred 플래그 유지 | tray_advanced 토픽 + 레이스 플래그 |
| 9 | HMI 크래시 | ROS 스레드에서 Qt 위젯 접근 | pyqtSignal 마샬링 |
| 10 | 재시작 타임아웃 | ros2_control 좀비 프로세스 | restart 스크립트에 kill 추가 |
| 11 | 보충 조기 종료 | 단일 프레임 NORMAL 오판정 | N회 연속 확인 |
| 12 | 외력 팝업 깜빡임 | 레벨 트리거 방식 | 상승 에지 래치 + acknowledged 플래그 |
