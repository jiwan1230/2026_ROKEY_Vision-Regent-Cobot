# Robot Control 슬라이드 제작 가이드라인 (슬라이드 11~18)

## 공통 슬라이드 스타일 (Vision 슬라이드와 동일하게 맞출 것)

- **헤더**: 진한 네이비 배경 스트립. 좌측 팀 번호(04), 우측 "K-Digital Training"
- **제목**: `[Robot]`, `[Gripper]`, `[Safety]` 카테고리 prefix 포함 굵은 제목
- **부제목**: 오렌지 또는 네이비 컬러의 한 줄 핵심 메시지
- **컨텐츠 박스**: 좌측 컬러 세로선 + 흰 박스 형태
  - 빨강 선: 문제/원인
  - 초록 선: 해결/효과
  - 파랑 선: 구조/개념
  - 노랑/주황 선: 주의/특이사항
- **테이블**: 헤더 행 네이비 배경, 흰 글자
- **플로우 화살표**: 둥근 모서리 직사각형 박스 + `▶` 또는 `→` 연결
- **푸터**: "ROKEY 협동1 · Vision Reagent Cobot" + 슬라이드 번호

---

## 시각화 전략 (전체 공통)

rqt_graph를 그대로 쓰지 말 것. 슬라이드마다 **해당 주제와 관련된 노드 2~4개만 추린 미니 다이어그램** 사용.

**색상 그룹 (노드 박스 배경색)**:
- 🔵 파랑: Vision 계열 (`liquid_height_detector_node`, `tube_state_publisher_node`, `side_camera_node`)
- 🟠 주황: Robot 제어 계열 (`robot_task_manager_node`, `doosan_robot_control_node`)
- 🟣 보라: 의사결정 (`main_decision_node`)
- 🟢 초록: Gripper (`gripper_control_node`)
- 🔴 빨강: Safety 관련 신호 강조
- ⬜ 회색: HMI (`hmi_node`), 하드웨어 드라이버 (`ros2_control_node`)

**토픽/서비스 표기**:
- 토픽: 실선 화살표 + `/topic_name` 레이블
- 서비스 호출: 점선 양방향 화살표 + `/service_name` 레이블

---

## Slide 11: [Robot] 노드 구성 및 작업 실행 구조

### 핵심 메시지
> Vision이 감지 → Main이 판단 → Task Manager가 실행 → 로봇/그리퍼가 동작

### 시각화: 전체 노드 맵 (유일하게 전체 그래프를 단순화해서 사용)

```
┌─────────────────────────────────────────────────────────────────┐
│                        🔵 Vision Layer                          │
│  [side_camera_node] → /vision/side_image                        │
│  [liquid_height_detector_node] → /vision/tube_height            │
│  [tube_state_publisher_node] → /vision/tube_state               │
└───────────────────────────┬─────────────────────────────────────┘
                            │ /vision/tube_state
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                     🟣 Decision Layer                            │
│              [main_decision_node]                                │
│   /vision/hand_detected ──▶  ──▶ /robot/start_task (서비스)    │
│   /robot/force_detected ──▶                                     │
└───────────────────────────┬─────────────────────────────────────┘
                            │ /robot/start_task (서비스 호출)
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                     🟠 Execution Layer                           │
│  [robot_task_manager_node]                                      │
│       │ /robot/move_to_pose (서비스)   │ /gripper/control (서비스)│
│       ▼                               ▼                         │
│  [doosan_robot_control_node]   [gripper_control_node]           │
│       │                               │                         │
│  dsr01 API (movel/movej)       Modbus TCP (192.168.1.1:502)    │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                     ⬜ HMI Layer                                 │
│  [hmi_node] — 모든 토픽/서비스 구독/제어 (PyQt5)               │
└─────────────────────────────────────────────────────────────────┘
```

### 텍스트 내용 (박스 구성)
- **🔵 파랑박스 - 노드 역할 표**

| 노드 | 역할 | 주요 출력 |
|------|------|-----------|
| side_camera_node | 카메라 영상 취득/압축 | /vision/side_image |
| liquid_height_detector_node | YOLO 추론, 높이 계산 | /vision/tube_height |
| tube_state_publisher_node | 상태 분류 (Refill/Normal/Dispose) | /vision/tube_state |
| main_decision_node | 우선순위 판단 → Task 지시 | /robot/start_task |
| robot_task_manager_node | 시퀀스 오케스트레이션 | /robot/move_to_pose |
| doosan_robot_control_node | DSR API 호출, 외력 감지 | /robot/force_detected |
| gripper_control_node | Modbus TCP 그리퍼 제어 | — |
| hmi_node | 운영자 인터페이스 (PyQt5) | — |

---

## Slide 12: [Robot] 우선순위 기반 Task Manager

### 핵심 메시지
> State2(폐기) > State0(보충) > State1(정상 이송) — 한 번에 하나의 우선순위만 처리, 완료 후 recheck

### 시각화: 결정 플로우차트 (세로 흐름)

```
[vision/tube_state 수신]
         │
         ▼
  UNKNOWN 포함? ──Yes──▶ [작업 보류]
         │ No
         ▼
  DISPOSE_NEEDED 있음? ──Yes──▶ [dispose_tube() 실행] ──▶ [request_recheck()]
         │ No                                                      │
         ▼                                                         │
  REFILL_NEEDED 있음? ──Yes──▶ [refill_tube() 실행] ──▶ [request_recheck()]
         │ No                                                      │
         ▼                                                         │
  전부 NORMAL? ──Yes──▶ [transfer_tray() 실행] ──▶ [_advance_tray()]
         │ No                                              │
         ▼                                         ← ─ ─ ─┘ (다음 사이클)
  [아무 작업 없음]
```

### 텍스트 내용
- **🔵 파랑박스 - 설계 원칙**
  - 한 번의 start_task 호출 = 한 우선순위 tier 처리
  - 작업 완료 후 반드시 recheck → vision 재평가 → 다음 tier 결정
  - tray_transferred 플래그로 동일 트레이 중복 이송 방지
  - _advance_tray() 후 /robot/tray_advanced 발행 → main_decision_node 플래그 리셋

- **🟠 주황박스 - 관련 노드 미니 다이어그램**
```
[main_decision_node] ──/robot/start_task──▶ [robot_task_manager_node]
        ▲                                            │
        └──────── /vision/request_recheck ───────────┘
```

---

## Slide 13: [Robot] Pose Catalog / Virtual TCP / Rotate Matrix

### 핵심 메시지
> 모든 동작 좌표를 YAML로 중앙 관리, TCP 오프셋 행렬 변환으로 정확한 위치 제어

### 시각화 (두 부분으로 구성)

**왼쪽: Pose Catalog 구조**
```
robot_params.yaml
├── home_pose: [492, 79, 220, 90, 180, -90]
├── tray_tool_stand_approach_pose: [350, -227, 220, ...]
├── tray_tool_stand_grip_pose: [...]
├── refill_source_approach_pose: [...]
├── refill_source_grip_pose: [...]
├── tray_transfer_tool_approach_pose(idx): [...]   ← 함수로 idx별 생성
├── tray_transfer_success_approach_pose(idx): [...]
├── dispose_tube_approach_pose(tray, tube): [...]
└── refill_target_approach_pose(tray, tube): [...]
```

**오른쪽: TCP / Rotate Matrix 개념도**
```
[플랜지 좌표계]
      │  apply_virtual_tcp()
      │  T_tcp = Rz(rz)·Ry(ry)·Rx(rx)
      │  flange_pos = TCP_target - R @ offset
      ▼
[TCP 끝단 좌표계]
  offset = [0, 25, 200] mm  ← 실측값
  기울인 pour_pose도 행렬로 자동 보정
```

### 텍스트 내용
- **🔵 파랑박스 - 설계 이유**
  - 좌표 하드코딩 제거 → YAML 한 곳에서 전체 관리
  - 로봇 교시(티칭) 값 변경 시 YAML만 수정
  - tray_idx / tube_idx 함수 파라미터로 다중 슬롯 자동 처리

- **🟠 주황박스 - TCP 오프셋 적용 이유**
  - 그리퍼 tool 교체 시 플랜지~끝단 거리 변경
  - pour 동작처럼 로봇이 기울면 단순 덧셈으로는 오차 발생
  - 회전 행렬로 모든 자세에서 정확한 TCP 위치 보장

---

## Slide 14: [Robot] Refill Sequence

### 핵심 메시지
> 시약통을 집어 tubes에 따르고, NORMAL 5회 연속 확인 후 복귀

### 시각화: 상세 시퀀스 플로우 (좌측) + 실제 사진 (우측)

```
[refill_tube(idx) 시작]
         │
         ▼
refill_source 위치로 이동
         │
         ▼
그리퍼 CLOSE (시약통 파지)
         │
         ▼
tube idx 위치로 이동 (approach → pour_pose)
         │
         ▼
┌────────────────────────────────┐
│       보충 루프 (최대 60s)      │
│  /robot/current_tube_state 조회 │
│         │                      │
│  NORMAL ──▶ consecutive+1      │
│  REFILL  ──▶ 시약통 rotate →  │
│              upright → pour    │
│  DISPOSE ──▶ TaskFailed (범람) │
│         │                      │
│  consecutive >= 5? ──Yes──▶ break│
└────────────────────────────────┘
         │
         ▼
시약통 세우기 (pour_pose → approach)
         │
         ▼
refill_source 반납 → 그리퍼 OPEN
         │
         ▼
HOME 복귀
```

### 텍스트 내용
- **🔴 빨강박스 - 문제**
  - height bbox가 1프레임 튀어 NORMAL로 오판 → 보충 조기 종료
- **🟢 초록박스 - 해결**
  - NORMAL 5회 연속 확인 (normal_confirm_count=5)
  - 최근 height 버퍼(중앙값) 기반 판단으로 순간 노이즈 제거

---

## Slide 15: [Robot] Dispose Sequence

### 핵심 메시지
> 과충전 tube를 집어 폐기 존에 버리고, handled_slot으로 NORMAL 취급

### 시각화: 시퀀스 플로우 (좌측) + 실제 사진 (우측)

```
[dispose_tube(idx) 시작]
         │
         ▼
tube idx 접근 (approach_pose)
         │
         ▼
하강 → 그리퍼 CLOSE (tube 파지)
         │
         ▼
상승 → 폐기 존 이동 (dispose_zone_pose)
         │
         ▼
그리퍼 OPEN (tube 투기)
         │
         ▼
/vision/mark_tube_disposed 호출
→ handled_slots[idx] = True
→ 이후 해당 슬롯은 confidence 무관 NORMAL 처리
         │
         ▼
HOME 복귀
```

### 텍스트 내용
- **🔵 파랑박스 - handled_slot 설계 이유**
  - 폐기 완료 후 빈 슬롯은 cup bbox 없음 → confidence 0 → UNKNOWN 오판 가능
  - 로봇이 직접 /vision/mark_tube_disposed 서비스 호출
  - 이후 해당 슬롯은 vision 결과와 무관하게 NORMAL로 처리
  - 트레이 교체(_advance_tray) 시 자동 리셋

---

## Slide 16: [Gripper] 완료 신호 기반 개폐 제어

### 핵심 메시지
> Modbus TCP raw socket으로 그리퍼 busy bit 폴링 — 동작 완료까지 블로킹

### 시각화: Modbus TCP 통신 타이밍 다이어그램

```
[gripper_control_node]           [그리퍼 컨트롤러 192.168.1.1:502]
         │                                    │
         │── FC3 Read Reg 268 (OPEN/CLOSE) ──▶│
         │                                    │  busy bit = 1 (동작 중)
         │◀─ Register value ─────────────────│
         │                                    │
         │  [10ms 간격 폴링 루프]              │
         │── FC3 Read Reg 268 ──────────────▶│
         │◀─ busy bit = 0 ──────────────────│  (완료)
         │                                    │
         │  → gripper_control 서비스 응답 반환│
         ▼                                    │
[robot_task_manager_node]
   grip() 리턴 후 다음 이동 진행
```

**Modbus 패킷 구조**:
```
MBAP Header(6B) + PDU
Transaction ID: 1
Protocol ID: 0
Unit ID: 65 (0x41)
Function Code: 3 (Read Holding Registers)
Register: 268 (0x010C)  ← busy bit 위치
Count: 1
```

### 텍스트 내용
- **🔴 빨강박스 - 문제**
  - pymodbus 라이브러리: ROS2 실행 환경에서 의존성 충돌 발생
- **🟢 초록박스 - 해결**
  - Python `socket` + `struct`로 Modbus TCP 패킷 직접 구현
  - 외부 라이브러리 의존 없이 raw socket 통신
  - 소켓 끊김 시 2회 자동 재연결 (auto-reconnect)
- **🔵 파랑박스 - grip_with_retry**
  - 그리퍼 CLOSE 후 busy=0이어도 실제로 물건을 잡았는지 불확실
  - 최대 3회 재시도 + 각 시도 후 별도 확인 로직

---

## Slide 17: [Safety] Hand / Emergency / Force Stop 구분

### 핵심 메시지
> 3단계 안전 정지: 손 감지(자동 재개) / 비상 정지(HMI 재개) / 외력 감지(HMI 재개)

### 시각화: 안전 계층 테이블 + 이벤트 플로우

**안전 계층 테이블**:

| 구분 | 트리거 | 정지 방식 | 재개 조건 | 해당 노드 |
|------|--------|-----------|-----------|-----------|
| Hand Stop | /vision/hand_detected=True | DR_HOLD (MoveStop) | 손 제거 자동 감지 | main_decision_node |
| Emergency Stop | HMI Emergency 버튼 | DR_HOLD + stop_event | HMI Start 버튼 | main_decision_node |
| Force Stop | GetToolForce > 20N | DR_HOLD + stop_event | HMI Start 버튼 | doosan_robot_control_node |

**이벤트 플로우 미니 다이어그램**:
```
[YOLO hand 감지]
      │ /vision/hand_detected
      ▼
[main_decision_node]
      │ stop_task(is_emergency=False)
      ▼
[robot_task_manager_node]
   stop_event.set()
      │
      ▼ (손 제거 감지)
   stop_event.clear() → 자동 재개
   
─────────────────────────────────

[외력 20N 초과]
      │ GetToolForce 서비스
      ▼
[doosan_robot_control_node]
   MoveStop(DR_HOLD)
   /robot/force_detected = True
      │
      ▼
[main_decision_node]
   stop_task(is_emergency=False)
      ← HMI Start 버튼으로만 해제 →
```

### 텍스트 내용
- **🔵 파랑박스 - stop_event vs emergency_event 구분**
  - stop_event: 손 감지 / 외력 / 비상 모두 공통으로 사용하는 정지 신호
  - emergency_event: HMI Emergency 전용 — pour 루프에서 세우기(upright) 후 시약통 반납
  - 외력: stop_event 해제는 HMI Start만 가능 (force_detected 플래그 유지)

---

## Slide 18: [Robot] Commit 기반 안정화 사례

### 핵심 메시지
> 실제 하드웨어 테스트에서 발견된 문제들을 커밋 단위로 추적하고 해결

### 시각화: Before/After 비교 카드 (4개)

**카드 1 — 시스템 재시작 후 로봇 무응답**
```
Before: Ctrl+C 후 재시작 → movel() 블로킹 좀비 프로세스 잔존
        → DDS stale 상태 → 서비스 응답 없음

After:  stop_event.set() in finally 블록
        → 블로킹 콜백 TaskAborted로 탈출
        restart_system.sh: ros2_control_node까지 포함 kill
```

**카드 2 — 트레이 이송 중 컵 충돌**
```
Before: lift_pose → success_approach 직행
        → 뒤쪽 트레이 컵에 충돌

After:  tray_idx=0,1: lift → TOOL_STAND → success
        tray_idx=2:   lift → tool_approach(0) → TOOL_STAND → success
```

**카드 3 — 2번 트레이 이송 후 시스템 정지**
```
Before: on_tray_advanced(tray_transferred=False)
        → on_start_task_done(tray_transferred=True) 3ms 후 덮어씀
        → 새 트레이 all_normal 영구 차단

After:  _tray_advanced_during_task 플래그 추가
        → on_start_task_done이 advance 발생 시 True 세우지 않음
```

**카드 4 — 로봇 팔을 hand로 오인식 → 비상정지 반복**
```
Before: hand bbox 전체 화면 무조건 감지
        → 그리퍼가 화면 상단(y<250px)에 진입 시 비상정지

After:  hand_roi_y_min_px=250 파라미터 추가
        → cy < 250px 박스는 안전 정지 트리거에서 제외
        → 오버레이에는 여전히 표시 (디버깅 가능)
```

### 텍스트 내용
- **🟠 주황박스 - 개발 방식**
  - 각 버그를 feature branch에서 수정 후 main 머지
  - 커밋 메시지에 원인/해결 명시 → git log로 추적 가능
  - 하드웨어 없이는 발견 불가능한 타이밍 이슈가 다수

---

## GPT에게 전달할 제작 지시사항

> 위 내용을 바탕으로 PowerPoint 슬라이드를 만들어줘.
> 
> **스타일 요구사항:**
> - 배경: 흰색
> - 상단 헤더 스트립: 네이비(#1B2A4A) 배경, 좌측 팀번호, 우측 "K-Digital Training" 흰색 텍스트
> - 제목: 굵게, 폰트 28~32pt
> - 컨텐츠 박스: 좌측 4px 세로선 + 연회색 배경, 선 색상은 빨강/초록/파랑/주황 구분
> - 다이어그램 박스: 둥근 모서리(8px), 텍스트 중앙 정렬
> - 화살표: 솔리드(토픽) / 점선(서비스 호출) 구분
> - 노드 색상: 파랑=Vision, 주황=Robot, 보라=Decision, 초록=Gripper, 회색=HMI/HW
> - 테이블: 헤더 행 네이비 배경 흰 글자, 짝수 행 연회색
> - 푸터: 좌측 "ROKEY 협동1 · Vision Reagent Cobot", 우측 슬라이드 번호
> 
> **슬라이드별 사진/영상 자리:**
> - 슬라이드 14(Refill): 📸 시약통 따르는 실제 사진
> - 슬라이드 15(Dispose): 📸 tube 폐기 동작 사진
> - 슬라이드 16(Gripper): 📸 그리퍼 클로즈 사진
> - 슬라이드 17(Safety): 📸 손 감지 화면 캡처
> - 슬라이드 18(안정화): 📸 테스트 현장 사진 또는 git log 캡처
