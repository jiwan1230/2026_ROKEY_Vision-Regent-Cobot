# Flowchart GPT 생성 프롬프트

아래 프롬프트들을 GPT(ChatGPT, Claude 등)에 붙여넣으면 Python 코드로 플로우차트 이미지를 생성해줍니다.
`matplotlib` + `patches` 기반 코드를 받아서 직접 실행하거나, GPT에게 실행까지 요청하세요.

---

## 프롬프트 1 — 전체 시스템 동작 흐름 (메인 사이클)

> 아래 내용을 GPT에 그대로 붙여넣으세요.

---

```
아래 설명을 바탕으로 Python matplotlib를 사용해서 플로우차트 이미지를 생성하는 코드를 작성해줘.
코드를 실행하면 PNG 파일로 저장되도록 해줘. 폰트는 영어로 작성해.

[플로우차트 제목]
Vision AI Reagent QC System — Main Operation Cycle

[스타일 가이드]
- 배경: 흰색
- 노드 모서리: 둥근 사각형(FancyBboxPatch)
- 색상 팔레트:
  · 시작/종료: 진한 남색 (#1A237E), 흰 글씨
  · Vision 처리 단계: 하늘색 (#E3F2FD 배경, #1565C0 테두리)
  · 판단(Decision) 다이아몬드: 주황색 (#FFF3E0 배경, #E65100 테두리)
  · 로봇 동작 단계: 초록색 (#E8F5E9 배경, #2E7D32 테두리)
  · 안전 정지: 빨간색 (#FFEBEE 배경, #C62828 테두리)
- 화살표: 진한 회색 (#424242), 굵기 1.5
- 폰트: 노드 내용 11pt, 제목 14pt bold

[플로우차트 노드 순서 및 내용]

1. START (시작, 남색 타원)
   ↓
2. "System Initialization\n(Camera / ROS2 Nodes / Robot)" (Vision 색)
   ↓
3. "HMI: Press START Button\n→ system_running = True" (Vision 색)
   ↓
4. "Vision Pipeline\nside_camera → YOLOv8n Inference\n→ Tube State Published" (Vision 색, 왼쪽 루프 화살표로 '계속 반복' 표시)
   ↓
5. 다이아몬드: "Safety Gate\nCamera OK?\nHand Detected?\nForce Detected?" (판단 색)
   → NO (오른쪽): "Hold & Wait\n(dispatch blocked)" (빨간색) → 4번으로 되돌아가는 화살표
   → YES (아래):
6. 다이아몬드: "Any State 2?\n(Overflow)" (판단 색)
   → YES (오른쪽): 7번
   → NO (아래): 8번

7. "DISPOSE Task\nPick Tube → Waste Zone\nReagent Dispose + Shake" (로봇 색)
   → 아래: "Request Recheck" (Vision 색) → 4번으로 되돌아가는 루프 화살표

8. 다이아몬드: "Any State 0?\n(Refill Needed)" (판단 색)
   → YES (오른쪽): 9번
   → NO (아래): 10번

9. "REFILL Task\nPick Reagent Tray\n→ Pour into Tube\n→ Return Tray" (로봇 색)
   → 아래: "Request Recheck" (Vision 색) → 4번으로 되돌아가는 루프 화살표

10. 다이아몬드: "All State 1?\n(All Normal)" (판단 색)
    → NO (오른쪽): "Idle — No Action" (회색 노드) → 4번 루프
    → YES (아래): 11번

11. "TRAY TRANSFER\nTray 0 → 1 → 2\n(Collision Avoidance Waypoints)" (로봇 색)
    ↓
12. "Task Complete\n→ Publish tray_advanced" (로봇 색)
    ↓
13. 다이아몬드: "All Trays Done?" (판단 색)
    → NO: 4번 루프
    → YES: END (남색 타원)

[추가 요소]
- 왼쪽에 세로로 "FORCE DETECTED" 인터럽트 표시:
  빨간 화살표가 5번 Safety Gate에서 별도로 나와서
  "External Force > 20N\n→ Stop Robot\n→ HMI Alert Popup\n→ Wait for START" 빨간 노드로 연결
  그 노드에서 다시 5번으로 되돌아가는 화살표

- 전체 크기: 가로 14인치, 세로 20인치
- 저장 파일명: flowchart_main_cycle.png, dpi=150
```

---

## 프롬프트 2 — 안전 시스템 상세 흐름

> 아래 내용을 GPT에 그대로 붙여넣으세요.

---

```
아래 설명을 바탕으로 Python matplotlib를 사용해서 플로우차트 이미지를 생성하는 코드를 작성해줘.
코드를 실행하면 PNG 파일로 저장되도록 해줘. 폰트는 영어로 작성해.

[플로우차트 제목]
Vision AI Reagent QC — Safety System Flow

[스타일 가이드]
- 배경: 흰색
- 3개 컬럼 레이아웃 (왼쪽: Hand Detection / 가운데: Normal Flow / 오른쪽: Force Detection)
- 색상:
  · 정상 흐름: 파란색 계열 (#E3F2FD / #1565C0)
  · 손 감지: 노란색 계열 (#FFFDE7 / #F9A825)
  · 외력 감지: 빨간색 계열 (#FFEBEE / #C62828)
  · 자동 재개: 초록색 계열 (#E8F5E9 / #2E7D32)
- 화살표: 각 컬럼 색상에 맞춤
- 크기: 가로 16인치, 세로 12인치

[플로우차트 구성]

[가운데 컬럼 — 정상 흐름]
Robot Task Running
↓
doosan_robot_control_node: GetToolForce polling (~7Hz)
↓ (항상)
/robot/force_norm published (Float64)
↓ (분기: force > 20N → 오른쪽 컬럼)
Publish /robot/force_detected = False
↓
main_decision_node: dispatch enabled
↓
robot_task_manager_node: Execute Task
↓ (루프)

[왼쪽 컬럼 — Hand Detection]
YOLOv8n detects 'hand' class
↓
ROI Filter: y_center >= 250px?
→ NO: Ignore (Robot Arm false positive)
→ YES:
/vision/hand_detected = True
↓
main_decision_node: call stop_task(stop=True)
↓
Robot Pauses
↓
Hand Cleared from Frame?
→ NO: Wait
→ YES:
call stop_task(stop=False)
↓
Robot Resumes Automatically
↓ (가운데 컬럼으로 복귀 화살표)

[오른쪽 컬럼 — Force Detection]
force_norm > 20N
↓
Publish /robot/force_detected = True
↓
main_decision_node: dispatch BLOCKED
robot_task_manager_node: stop_event SET
↓
Robot Stops Immediately
↓
HMI Force Alert Popup (Latched)
↓
Operator Inspects & Removes Hazard
↓
HMI: Press START Button
↓
force_detected = False (reset)
dispatch re-enabled
↓ (가운데 컬럼으로 복귀 화살표)
(No Auto-Resume — Manual Only)

[하단 범례]
- 손 감지: 자동 재개 O
- 외력 감지: 수동 재개만 가능 (START 버튼)
- Emergency Stop 버튼: 즉시 stop_task(is_emergency=True) 호출

저장 파일명: flowchart_safety_system.png, dpi=150
```

---

## 프롬프트 3 — 트레이 이송 충돌 회피 경로

> 아래 내용을 GPT에 그대로 붙여넣으세요.

---

```
아래 설명을 바탕으로 Python matplotlib를 사용해서 플로우차트 이미지를 생성하는 코드를 작성해줘.
코드를 실행하면 PNG 파일로 저장되도록 해줘. 폰트는 영어로 작성해.

[플로우차트 제목]
Tray Transfer — Collision Avoidance Waypoints

[스타일 가이드]
- 배경: 흰색
- 2개 컬럼 레이아웃
  · 왼쪽 컬럼: Tray 0 / Tray 1 경로 (파란색 계열)
  · 오른쪽 컬럼: Tray 2 경로 (보라색 계열 #EDE7F6 / #4527A0)
- 공통 경유지(waypoint): 진한 회색 (#455A64) 강조
- 충돌 회피 경유지: 주황색 테두리 (#E65100) 강조
- 크기: 가로 14인치, 세로 14인치

[왼쪽 컬럼 — Tray 0 & 1]

tray_transfer_tool_approach_pose(tray_idx)
→ Gripper CLOSE (grasp tray)
→ tray_transfer_lift_pose(tray_idx)   ← 트레이 들어올림
→ [충돌 회피 경유지 1] TRAY_TOOL_STAND_APPROACH_POSE
  (뒤쪽 트레이 컵 충돌 방지 — 주황 강조)
→ tray_transfer_success_approach_pose(tray_idx)
→ 성공 구역 안착
→ Gripper OPEN
→ /robot/tray_advanced 발행

[오른쪽 컬럼 — Tray 2 only]

tray_transfer_tool_approach_pose(2)
→ Gripper CLOSE (grasp tray)
→ tray_transfer_lift_pose(2)   ← 트레이 들어올림
→ [충돌 회피 경유지 A] tray_transfer_tool_approach_pose(0)
  (1번 성공 트레이와 충돌 회피 — 주황 강조)
→ [충돌 회피 경유지 B] TRAY_TOOL_STAND_APPROACH_POSE
  (뒤쪽 트레이 컵 충돌 방지 — 주황 강조)
→ tray_transfer_success_approach_pose(2)
→ 성공 구역 안착
→ Gripper OPEN
→ /robot/tray_advanced 발행

[하단 설명 텍스트 박스]
- Tray 0, 1: 경유지 1개 (TOOL_STAND)
- Tray 2: 경유지 2개 (tool_approach(0) → TOOL_STAND)
  이유: 1번 트레이를 이미 성공 구역으로 옮긴 후 2번 이송 시
        1번 트레이와의 충돌 가능성이 있어 우회 경로 추가

저장 파일명: flowchart_tray_transfer.png, dpi=150
```

---

## 사용 방법

1. 위 프롬프트 중 원하는 것을 **통째로 복사**
2. GPT(ChatGPT 4o 이상 권장)에 붙여넣기
3. GPT가 Python 코드를 생성하면:
   - **GPT 내에서 실행 요청**: "이 코드를 실행해서 이미지 파일을 생성해줘"
   - **로컬 실행**: 코드를 `flowchart.py`로 저장 후 `python3 flowchart.py`
4. 수정이 필요하면: "노드 색상을 바꿔줘", "화살표 방향을 수정해줘" 등 후속 요청

## 팁

- GPT가 한 번에 완벽하게 안 나올 수 있습니다. "레이아웃이 겹쳐보여, 간격을 넓혀줘" 처럼 구체적으로 피드백하세요.
- Mermaid.js 버전을 원한다면: 프롬프트 맨 앞에 "matplotlib 대신 Mermaid.js 코드로 작성해줘"를 추가하면 [mermaid.live](https://mermaid.live)에서 바로 렌더링 가능합니다.
- 발표용 고해상도가 필요하면: `dpi=300`으로 수정 요청하세요.
