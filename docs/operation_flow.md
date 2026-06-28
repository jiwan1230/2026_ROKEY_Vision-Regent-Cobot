# 전체 동작 순서도

## 메인 사이클 흐름

```
 [시스템 시작]
      │
      ▼
 노드 초기화 (카메라, ROS2, 로봇, 그리퍼 Modbus 연결)
      │
      ▼
 HMI "Start" 버튼 → system_running=True
      │
      ▼
 ┌────────────────────────────────────────────────┐
 │          Vision 파이프라인 (연속 반복)           │
 │  side_camera_node → CompressedImage 발행        │
 │  liquid_height_detector_node (YOLOv8n 추론)     │
 │   → tube_height, hand_detected, camera_status   │
 │  tube_state_publisher_node                      │
 │   → tube_state (State 0/1/2/UNKNOWN)            │
 └───────────────────────┬────────────────────────┘
                         │ /vision/tube_state
                         ▼
              [main_decision_node 안전 게이트]
                         │
          ┌──────────────┼──────────────────┐
          │              │                  │
    camera_ok?    hand_detected?    force_detected?
      NO→보류        YES→보류          YES→보류
          │              │                  │
          └──────────────┴──────────────────┘
                         │ 모두 통과
                         ▼
              [robot_task_manager_node 우선순위 분기]
                         │
           ┌─────────────┼─────────────┐
           │             │             │
       State 2?      State 0?    All State 1?
         YES            YES          YES
           │             │             │
           ▼             ▼             ▼
       [폐기]         [보충]       [트레이 이송]
```

---

## 세부 동작: 폐기 (State 2 — Dispose)

```
 tube_X 폐기 대상 확인
      │
      ▼
 home_pose → tube_X_approach_pose → tube_X_pick_pose
      │
      ▼
 그리퍼 CLOSE (Modbus 폴링으로 grip_detected 확인)
      │
      ▼
 tube_X_pick_pose → tube_X_approach_pose (lift)
      │
      ▼
 waste_approach_pose → waste_pose
      │
      ▼
 [시약 분주 (dispose)]
  reagent_dispose: 회전(rotate) → 털기(shake) → 원위치
  tube_dispose:    튜브 내용 비우기 동작
      │
      ▼
 그리퍼 OPEN
      │
      ▼
 /vision/mark_tube_disposed → /vision/request_recheck
      │
      ▼
 home_pose → 다음 사이클 대기
```

---

## 세부 동작: 보충 (State 0 — Refill)

```
 tube_X 보충 대상 확인
      │
      ▼
 home_pose → tray_pick_approach → tray_pick_pose
      │
      ▼
 시약 트레이(reagent) 파지 (그리퍼 CLOSE)
      │
      ▼
 tray_pick_pose → tube_X_approach_pose → tube_X_refill_pose
      │
      ▼
 [보충 루프]
  refill_pose에서 기울이기 → /robot/current_tube_state 폴링
  State 1(NORMAL) 연속 N회 확인 시 루프 종료
      │
      ▼
 tube_X_refill_pose → tube_X_approach_pose → tray_pick_pose
      │
      ▼
 그리퍼 OPEN (시약 트레이 복귀)
      │
      ▼
 /vision/request_recheck → 다음 사이클
```

---

## 세부 동작: 트레이 이송 (All Normal — Transfer)

```
 tray_idx = 0, 1, 2 순서로 순차 처리
      │
      ▼
 home_pose → tray_tool_approach_pose(tray_idx)
      │
      ▼
 그리퍼 CLOSE (트레이 파지)
      │
      ▼
 tray_transfer_lift_pose(tray_idx)  ← 트레이 들어올림
      │
      ├── [tray_idx >= 2] ──► tray_transfer_tool_approach_pose(0)
      │                             ↓  (1번 성공 트레이 충돌 회피)
      │
      ▼
 TRAY_TOOL_STAND_APPROACH_POSE  ← 뒤쪽 트레이 컵 충돌 방지 경유지
      │
      ▼
 tray_transfer_success_approach_pose(tray_idx) → 성공 구역 안착
      │
      ▼
 그리퍼 OPEN
      │
      ▼
 /robot/tray_advanced 발행 (tray_idx+1)
      │
      ▼
 모든 트레이 완료 시 → 작업 종료
```

---

## 외력 감지 안전 인터럽트

```
 doosan_robot_control_node: GetToolForce 폴링 (약 7Hz)
      │
      ▼
 force_norm > 20N ?
      │ YES
      ▼
 /robot/force_detected = True 발행
      │
      ▼
 main_decision_node: force_detected = True → dispatch 보류
 robot_task_manager_node: stop_event 세팅 → 진행 중 동작 정지
 hmi_node: Force Alert 팝업 표시
      │
      ▼
 작업자 확인 후 HMI "Start" 버튼 클릭
      │
      ▼
 force_detected = False (리셋) → 자동 dispatch 재개
```

---

## 시나리오 매핑

| 시나리오 | 입력 예시 | 처리 순서 |
|----------|-----------|-----------|
| A: 일부 보충 필요 | `[0, 1, 1]` | refill(0) → recheck → (정상) → tray transfer |
| B: 일부 폐기 필요 | `[1, 2, 1]` | dispose(1) → recheck |
| C: 전체 정상 | `[1, 1, 1]` | tray transfer (tray 0→1→2 순서) |
| D: 보충+폐기 동시 | `[0, 2, 1]` | dispose(1) → recheck → refill(0) → recheck → tray transfer |
| E: 외력 감지 발생 | 작업 중 충돌 | 즉시 정지 → 팝업 → 작업자 Start → 재개 |

매 사이클마다 최고 우선순위 1건만 처리(State2 > State0 > State1). Vision 파이프라인은 멈추지 않으므로 recheck 없이도 다음 `/vision/tube_state`가 자동으로 다음 사이클을 트리거한다.
