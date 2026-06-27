# Vision AI 기반 시약 자동 관리 협동로봇 시스템
## 발표 자료 초안

> **편집 안내**
> - 이 파일은 Markdown 형식입니다.
> - VS Code에서 편집 후 `Markdown PDF` 익스텐션으로 PDF 변환 가능
> - Pandoc 명령: `pandoc presentation_draft.md -o presentation.pdf --pdf-engine=xelatex -V mainfont="NanumGothic"`
> - 또는 [HackMD](https://hackmd.io), [Notion](https://notion.so) 에 붙여넣어 편집 가능
> - `📸` 표시 자리에 실제 사진/영상 캡처를 삽입하세요

---

## 슬라이드 1 — 표지

**제목:** Vision AI 기반 시약 자동 관리 협동로봇 시스템

**부제:** YOLOv8 + ROS2 + Doosan M0609 통합 자동화 파이프라인

| 항목 | 내용 |
|------|------|
| 팀명 | |
| 발표일 | 2026년 6월 |
| 소속 | |

> 📸 **사진 자리:** 실제 로봇 + 트레이 전경 사진

---

## 슬라이드 2 — 팀 소개 / 역할 분담

| 이름 | 역할 | 주요 담당 |
|------|------|-----------|
| | Vision AI | YOLO 학습, 라벨링, 추론 파이프라인 |
| | Robot Control | ROS2 로봇 노드, 모션 시퀀스 |
| | HMI | PyQt5 대시보드, 관리자 탭 |
| | Integration / Safety | 외력 감지, 시스템 통합 |

---

## 슬라이드 3 — 프로젝트 배경 및 필요성

### 기존 문제

- 실험실 시약 보충·폐기 작업의 **100% 수작업** 의존
- 육안으로 액체 잔량 확인 → 개인 오차 발생
- 화학물질 직접 취급으로 인한 **안전 위험**
- 반복 작업으로 인한 피로 누적 및 실수 증가

### 해결 방향

- 카메라 + AI로 시약 잔량을 **자동 측정**
- 상태 판정 결과를 협동로봇이 즉시 **후속 조치 수행**
- 작업자는 HMI로 모니터링만 → **안전 + 효율 동시 확보**

> 📸 **사진 자리:** 시약통 트레이 배치 사진 / 작업 전 상황 사진

---

## 슬라이드 4 — 시스템 개요

### 한 줄 요약

> 카메라가 **보고** → AI가 **판단하고** → 로봇이 **행동한다**

```
카메라 → YOLO 추론 → 상태 분류 → ROS2 통신 → 로봇 제어
                                              ↕
                                         HMI 모니터링
```

### 3가지 동작 모드

| 상태 | 의미 | 로봇 동작 |
|------|------|-----------|
| **State 0** | 시약 부족 | 보충 (Refill) |
| **State 1** | 정상 | 트레이 이송 (Normal Transfer) |
| **State 2** | 과잉 / 불량 | 폐기 (Dispose) |
| **UNKNOWN** | 검출 실패 | 작업 보류, HMI 알림 |

---

## 슬라이드 5 — 개발 환경

### Hardware

| 장비 | 사양 |
|------|------|
| 협동로봇 | Doosan Robotics M0609 (반경 900mm, 6-DOF) |
| 카메라 | Side-view USB 카메라 1대 |
| PC (Vision) | YOLOv8n CPU/GPU 추론 가능 노트북 |
| PC (Main) | ROS2 Humble 실행 + HMI 표시 |

### Software

| 항목 | 기술 스택 |
|------|-----------|
| OS | Ubuntu 22.04 |
| Robot Framework | ROS2 Humble + DSR_MSGS2 |
| 언어 | Python 3.10 |
| Vision AI | YOLOv8n (Ultralytics), OpenCV |
| HMI | PyQt5 |
| 통신 | Modbus TCP (그리퍼), ROS2 Topic/Service |
| 버전 관리 | GitHub (브랜치 전략 기반) |

---

## 슬라이드 6 — 전체 시스템 아키텍처

```
┌─────────────┐
│ Side Camera │
└──────┬──────┘
       │ /vision/side_image (JPEG Compressed, ~10KB)
       ▼
┌─────────────────────────────┐
│  LiquidHeightDetectorNode   │──→ /vision/yolo_result_image (HMI)
│  (YOLOv8n: cup/height/hand) │──→ /vision/hand_detected     (안전)
└──────────────┬──────────────┘──→ /vision/camera_status     (watchdog)
               │ /vision/tube_height
               ▼
┌──────────────────────────┐
│  TubeStatePublisherNode  │
│  (State 0 / 1 / 2 / -1) │
└─────────────┬────────────┘
              │ /vision/tube_state
              ▼
┌───────────────────────┐   HMI Start/Stop
│   MainDecisionNode    │◄──────────────────
│  (안전 게이트 + 트리거) │
└──────────┬────────────┘
           │ /robot/start_task
           ▼
┌────────────────────────────┐
│  RobotTaskManagerNode      │──→ /robot/status
│  (우선순위 작업 시퀀스)      │──→ /robot/force_detected
└───┬──────────────┬─────────┘──→ /robot/force_norm
    │              │
    ▼              ▼
DoosanRobotControlNode   GripperControlNode
(M0609 실제 제어)         (Modbus TCP)
    │
    ▼
┌───────────────────────────────┐
│       HMI Dashboard (PyQt5)   │
│  카메라뷰 / 상태 / 로그 / 제어  │
└───────────────────────────────┘
```

> 📸 **사진 자리:** `ros2 run rqt_graph rqt_graph` 스크린샷

---

## 슬라이드 7 — 개발 타임라인 (Git 기반)

```
06/23  ████ 초기 스캐폴딩 + YOLOv8n 번들링 + SETUP.md
06/24  █████████ Vision 파이프라인 + ONNX 변환 + 로봇 제어 1차 + HMI Tkinter
06/24  ████ PyQt5 전환 + Vision 안정화 (JPEG 압축, slot anchor)
06/25  █████████████ 로봇 제어 2차 + 포즈 카탈로그 + 손 감지 + 비상정지
06/25  ████ 외력 감지 초안 (feature/force-detection 브랜치)
06/26  ████████ 폐기 시퀀스 개선 + HMI 로그 + 외력 보정
06/27  ████████████ HMI 전면 개편 + Vision 데드코드 정리 + 외력 캘리브레이션
06/28  ████ 브랜치 통합 완료 (main 머지)
```

### 주요 브랜치 (7개)

| 브랜치 | 목적 |
|--------|------|
| `fix/tcp-offset-shape` | 가상 TCP shape mismatch 버그 수정 |
| `fix/gripper-modbus-wait` | Modbus TCP 그리퍼 상태 확인 구현 |
| `feature/force-detection` | 관절 토크 기반 외력 감지 초안 |
| `feature/force-calibration` | TCP 힘 기반으로 전환 + 임계값 보정 |
| `26_rotate_matrix` | TCP 회전 행렬 적용 |
| `27_dispose_update` | 폐기 시퀀스 안정화 |
| `hmi_node_part` | HMI 전면 개편 (관리자 탭, 파라미터) |

---

## 슬라이드 8 — [Vision] YOLO 모델 학습 및 Auto-Labeling

### 학습 목표

시약통 측면 이미지에서 3개 클래스 동시 검출:
- `cup` — 시약통 전체 영역
- `height` — 액체 채워진 영역
- `hand` — 작업자 손 (안전 감지용)

### Auto-Labeling 워크플로우

```
소량 수동 라벨링 (seed data)
         ↓
YOLOv8n 초기 학습
         ↓
미라벨 이미지에 모델 예측 적용 (Auto-label)
         ↓
예측 결과 검토 / 수정 (사람이 빠르게 확인만)
         ↓
전체 데이터셋으로 재학습
         ↓
ONNX 변환 (reagent_yolov8n.onnx) → 플랫폼 독립 배포
```

### 효과

- 라벨링 공수 대폭 절감 (수동 대비 ~70% 시간 단축 추정)
- YOLOv8n 선택 이유: **CPU 추론 가능** (별도 GPU 불필요)

> 📸 **사진 자리:** 라벨링 화면 스크린샷 / YOLO 추론 결과 오버레이 이미지

---

## 슬라이드 9 — [Vision] 추론 파이프라인 핵심 기술

### 문제 1: cv_bridge ABI 충돌

- **원인:** pip numpy 2.x ↔ apt cv_bridge 세그폴트
- **해결:** `ros_image_utils.py` 직접 구현 (cv_bridge 미사용)

### 문제 2: 이미지 전송 드롭

- **원인:** Raw BGR 이미지 921KB → BEST_EFFORT QoS에서 드롭
- **해결:** JPEG 압축 `CompressedImage` 전환 → **~10KB** (99% 감소)

### 문제 3: 시약통 슬롯 위치 불안정

- **원인:** 프레임마다 좌/중/우 cup 매핑이 흔들림
- **해결:** **Slot Anchor 시스템** — 첫 안정 프레임에서 anchor 부트스트랩, 이후 tolerance 범위 내 매칭 유지

### 문제 4: 오른쪽 영역 오검출

- **원인:** 성공 트레이가 카메라 시야에 겹쳐 보임
- **해결:** ROI x축 필터 (`roi_x_max_px: 540`) — **hand 감지는 예외** (안전 기능은 전체 화면 커버)

> 📸 **사진 자리:** YOLO 추론 결과 박스 오버레이 화면 / ROI 필터 적용 비교

---

## 슬라이드 10 — [Vision] 액체 높이 측정 원리

### 측정 방식

```
┌─────────────────────────┐
│      cup bbox           │  ← YOLO "cup" 클래스
│                         │    (시약통 전체 영역)
│   ┌──────────────┐      │
│   │  height bbox │      │  ← YOLO "height" 클래스
│   │              │      │    (액체 채워진 영역)
│   └──────────────┘      │
│                         │
└─────────────────────────┘

liquid_fill_fraction = height_bbox_높이 / cup_bbox_높이
                     = 0.0 ~ 1.0 (= 0% ~ 100%)
```

### 안정화 처리

| 처리 | 설정값 | 효과 |
|------|--------|------|
| Median 필터 | 최근 5프레임 | 순간 오차 제거 |
| 최소 샘플 | 3샘플 이상 | 초기 불안정 제거 |
| 연속 NORMAL 확인 | 5회 연속 | 보충 조기 종료 방지 |

---

## 슬라이드 11 — [Robot] 노드 구성 및 역할

| 노드 | 역할 |
|------|------|
| `main_decision_node` | 비전 상태 구독 → 안전 게이트 → 작업 트리거 |
| `robot_task_manager_node` | 우선순위 기반 작업 시퀀스 실행 |
| `doosan_robot_control_node` | M0609 실제 제어 (movel, MoveJ, 외력 감지) |
| `gripper_control_node` | Modbus TCP 기반 그리퍼 개폐 제어 |

### 작업 우선순위 원칙

```
우선순위: 폐기(State 2) > 보충(State 0) > 정상이송(State 1)

한 사이클에 최우선 1건만 처리
       ↓
  Recheck 요청
       ↓
 다음 사이클에서 재평가
```

> 동시에 여러 상태가 존재해도 항상 올바른 순서 보장

---

## 슬라이드 12 — [Robot] 포즈 카탈로그 & 가상 TCP

### 포즈 관리 방식

- 모든 좌표는 `robot_params.yaml`에 **named pose**로 등록
- 형식: `[x, y, z, rx, ry, rz]` Doosan posx 규약 (mm, deg ZYZ euler)
- 코드에는 **pose_name 문자열만** 전달 → 좌표 변경이 YAML 수정으로 완결
- HMI 관리자 탭에서 런타임 수정 가능

### 가상 TCP (Virtual TCP)

- 그리퍼 끝점 오프셋을 소프트웨어로 적용
- TCP 회전 행렬 (`26_rotate_matrix` 브랜치)로 실제 공구 끝 좌표 변환
- `fix/tcp-offset-shape` 브랜치에서 shape mismatch 버그 수정

```python
# TCP 오프셋 적용 전
pose = [x, y, z, rx, ry, rz]

# TCP 오프셋 적용 후 (회전 행렬 변환)
pose_tcp = apply_virtual_tcp(pose, tcp_offset[:3])
```

> 📸 **사진 자리:** 실제 로봇 동작 + 포즈 위치 다이어그램

---

## 슬라이드 13 — [Robot] Refill (보충) 시퀀스

```
home_pose
    ↓
refill_source_approach_pose    (시약 소스 접근)
    ↓
refill_source_grip_pose        (그리퍼 닫기 + Modbus 완료 확인)
    ↓
refill_target_N_approach_pose  (대상 시약통 접근)
    ↓
refill_target_N_work_pose      (작업 위치)
    ↓
┌── refill_target_N_pour_pose  (기울이기 → 높이 확인 루프) ──┐
│   NORMAL 5회 연속 확인 → 완료                              │
└────────────────────────────────────────────────────────────┘
    ↓
원위치 복귀
    ↓
home_pose
```

**안전 처리**
- 보충 중 손 감지 → 병 세우기 후 일시정지 → 안전 확인 후 재개
- `max_pour_attempts: 20` — 무한루프 안전 상한
- `TaskAborted` / `TaskFailed` 예외 → 항상 home_pose 복귀 보장

> 📸 **사진 자리:** 보충 동작 시퀀스 영상 캡처

---

## 슬라이드 14 — [Robot] Dispose (폐기) 시퀀스

```
home_pose
    ↓
dispose_tray_N_tube_M_approach_pose  (시약통 접근)
    ↓
dispose_tray_N_tube_M_grip_pose      (시약통 파지)
    ↓
waste_approach_pose                  (폐기 구역 이동)
    ↓
waste_release_pose                   (시약통 내려놓기)
    ↓
waste_rotate_pose                    (뒤집기 + 털기 동작)
    ↓
waste_rotate_middle_pose → home_pose (원위치 복귀)
```

**개발 이슈 및 해결**

| 문제 | 해결 |
|------|------|
| 폐기→원위치 중 재정지 발생 | 1회 실행으로 회전+원위치 처리 (`27_dispose_update`) |
| 정지 시 그리퍼가 공중에서 열림 | `_do_cleanup` 별도 패턴 도입 |
| dispose: reagent vs tube 구분 불명확 | reagent dispose / tube dispose 두 경로로 분할 |

> 📸 **사진 자리:** 폐기 동작 시퀀스 영상 캡처

---

## 슬라이드 15 — [Safety] 다층 안전 시스템

### Layer 1 — 손 감지 (Vision)

- YOLOv8n `hand` 클래스 실시간 검출 (ROI 무관 전체 화면)
- **N프레임 연속 감지** → `/robot/stop_task` (즉시 정지)
- **M프레임 연속 소실** 후 해제 (오검출 방지)
- HMI에서 활성화/비활성화 토글 가능

### Layer 2 — 외력 감지 (Force)

- `GetToolForce` 서비스로 TCP Cartesian 힘 측정
- 수식: `||F|| = √(Fx² + Fy² + Fz²)`
- 임계값 초과 → `MoveStop(DR_HOLD)` → HMI 알람
- HMI Start 버튼으로만 해제 (담당자 확인 의미)

### Layer 3 — 카메라 Watchdog

- 3초 이상 프레임 미수신 → `/vision/camera_status = False`
- `main_decision_node` 자동 작업 보류

### Layer 4 — HMI 비상정지 버튼

- 항상 최상단 노출, 즉시 `/robot/stop_task` 호출
- Start 버튼으로만 재개

---

## 슬라이드 16 — [Safety] 외력 감지 개발 과정

### 1단계: GetExternalTorque (관절 토크 기반)

```
문제 1: 포즈마다 자세 부하가 달라 임계값 설정 어려움
        (팔이 뻗으면 중력 모멘트가 커져서 정상값이 변함)
문제 2: _force_pending 가드 → 서비스 응답 지연 시
        timer의 모든 tick이 skip → force_norm 토픽 동결
```

### 2단계: GetToolForce (TCP Cartesian 힘 기반) — `feature/force-calibration`

```
장점: 페이로드 보상이 컨트롤러 수준에서 처리됨
      → 포즈 의존성 없이 안정적 측정 (정상: 0~3 N)

개선:
  ① GetExternalTorque → GetToolForce 서비스 전환
  ② force_offset 캘리브레이션 파라미터 완전 제거
  ③ _force_pending 가드 완전 제거 → 토픽 일정 주기 갱신 복원
  ④ 임계값: 20 N 설정
```

**결론:** 단위가 Nm(관절) → N(TCP)으로 바뀌고, 포즈 의존성이 사라져 임계값 설정이 단순해짐

---

## 슬라이드 17 — [Gripper] Modbus TCP 연동

### 문제

그리퍼 OPEN/CLOSE 완료를 확인할 방법 부재
- 기존: 디지털 입력 폴링 → 응답 불안정
- 결과: 파지 전에 다음 동작 진행 → 시약통 낙하 위험

### 해결 (`fix/gripper-modbus-wait` 브랜치, PR #3 / #5)

```python
# 기존: 단순 time.sleep()
gripper.close()
time.sleep(1.0)  # 대충 기다림

# 개선: Modbus TCP 레지스터로 완료 신호 확인
gripper.close()
while not modbus_read_grip_status():  # 실제 완료 신호
    time.sleep(0.05)
```

- 실제 파지 완료 신호 수신 후 다음 동작 진행
- 불필요한 `time.sleep` 제거 → 사이클 타임 단축

> 📸 **사진 자리:** 그리퍼 동작 영상 캡처

---

## 슬라이드 18 — [HMI] 개발 과정

### 1단계 (06/24): Tkinter 기반 초안

기본 상태 표시 + 수동 제어 버튼

### 2단계 (06/24): PyQt5 전환

- **전환 이유:** 스레드 안전성, 풍부한 위젯, 레이아웃 유연성
- ROS 콜백(별도 스레드)에서 Qt 위젯 직접 접근 → 크래시
- 해결: **`pyqtSignal`** 로 GUI 스레드에 marshal

### 3단계 (06/25): 기능 확장

- JOG 탭 추가 (수동 로봇 이동)
- 관리자 탭 (로그인 보호, 파라미터 설정)
- Vision Log 탭 (별도 `/vision/log` 토픽)
- 비상정지 즉시 동작 개선

### 4단계 (`hmi_node_part` 브랜치): 전면 개편

- 파라미터 실시간 읽기/쓰기 (YAML → ROS2 서비스)
- DR-M0609 물리 범위 검증 (Apply 시 체크)
- `collections.deque` 메모리 누수 수정
- `pyqtSignal` 패턴 확장 적용

---

## 슬라이드 19 — [HMI] 화면 구성

> 📸 **사진 자리:** HMI 전체 화면 스크린샷 (각 탭별)

| 탭 | 주요 내용 |
|----|-----------|
| **메인** | 카메라 뷰 + 튜브 상태 LED + 로봇 상태 |
| **모션 제어** | 속도/가속도 파라미터 + 손 감지 안전 토글 |
| **JOG** | 수동 로봇 이동 (관리자 전용) |
| **관리자** | 포즈 파라미터 설정 + 시스템 파라미터 |
| **Robot Log** | 로봇 작업 이벤트 로그 |
| **Vision Log** | Vision 추론/카메라 이벤트 로그 |

**공통 요소**
- 비상정지 버튼: 항상 최상단 노출, 즉시 동작
- 외력 감지 알람: 팝업 + 로그 동시 표시
- Start / Stop: 자동 루프 게이트 제어

---

## 슬라이드 20 — 주요 버그 해결 사례

| # | 문제 | 원인 | 해결 |
|---|------|------|------|
| 1 | HMI Qt 크래시 | ROS 콜백에서 직접 Qt 위젯 접근 | `pyqtSignal`로 GUI 스레드 marshal |
| 2 | 비상정지 후 멈춤 | `_call_sync` 내부 중첩 executor | `stop_event.set()` 즉시 반환 패턴 |
| 3 | refill 조기 종료 | YOLO 순간 오차로 NORMAL 오판 | N회 연속 확인 (`normal_confirm_count: 5`) |
| 4 | 그리퍼 공중 열림 | 정지 시 cleanup 분기 누락 | `_do_cleanup` 별도 패턴 |
| 5 | 손 감지로 작업 abort | pour 루프 내 stop 처리 분기 누락 | hand-stop vs emergency-stop 구분 |
| 6 | force_norm 토픽 동결 | `_force_pending` 가드가 모든 tick skip | 가드 완전 제거 |
| 7 | 관리자 탭 Enter 키 버그 | dialog keyPressEvent 미처리 | Accept 시그널 연결 |
| 8 | `spin_thread.join()` 충돌 | shutdown 순서 오류 | `shutdown → join → destroy_node` 순서 고정 |

---

## 슬라이드 21 — 시스템 동작 시연 흐름

### 시나리오: tube 0(부족) / tube 1(정상) / tube 2(과잉)

```
[1] 시스템 시작
    → 카메라 연결, 노드 초기화, HMI 기동

[2] 초기 상태 판정
    → YOLO 추론 → tube 0: State 0 / tube 1: State 1 / tube 2: State 2

[3] 우선순위 처리 — State 2 먼저 (폐기)
    → tube 2 파지 → 폐기 구역 이동 → 털기 → 원위치

[4] Recheck → State 0 처리 (보충)
    → tube 0 접근 → 시약 소스 파지 → 보충 → NORMAL 5회 확인

[5] Recheck → 전체 State 1
    → 트레이 전체 정상 구역 이송

[6] 안전 기능 시연
    → 손을 카메라 앞에 → 즉시 정지 확인
    → 외력 인가 → force_norm 상승 → 정지 + 알람
```

> 📸 **영상 자리:** 전체 동작 시연 영상 / 안전 기능 시연 영상

---

## 슬라이드 22 — 결과 및 성과

### 정량적 결과

| 항목 | 결과 |
|------|------|
| 라벨링 공수 | Auto-labeling으로 ~70% 절감 |
| 이미지 전송 용량 | 921KB → ~10KB (JPEG, 99% 감소) |
| 액체 높이 측정 정확도 | median 필터 후 ±5% 이내 |
| 외력 감지 임계값 | 20 N (정상 동작 중 0~3 N) |

### 정성적 결과

- 보충 / 폐기 / 정상이송 **3개 시나리오 전자동** 수행
- 손 감지 + 외력 감지 **이중 안전망** 구현
- HMI 통한 **실시간 모니터링** 및 파라미터 런타임 수정
- Git 브랜치 전략으로 **병렬 기능 개발 + 안전한 통합**

> 📸 **사진 자리:** 결과 비교 사진 / 시스템 동작 중 HMI 화면

---

## 슬라이드 23 — 향후 개선 사항

| 분야 | 개선 방향 |
|------|-----------|
| Vision | 조명 변화 대응 (HSV 정규화), 다각도 카메라 추가 |
| Robot | 포즈 티칭 자동화 (비전 기반 캘리브레이션) |
| Safety | 3D 공간 occupancy 기반 충돌 예측 |
| HMI | 웹 기반 대시보드 (원격 모니터링) |
| 모델 | 시약 종류 분류 클래스 추가 (라벨 확장) |
| 통합 | 다관절 트레이 자동 교체 로직 |

---

## 슬라이드 24 — 마무리 / Q&A

### 이 프로젝트가 보여준 것

> 단순한 비전 검출을 넘어,
> **판정 → ROS2 통신 → 로봇 후속 조치** 가 자동으로 순환하는
> 완전한 자동화 파이프라인의 구현

- ROS2 Topic/Service 모델로 이기종 컴포넌트를 깔끔하게 연결
- Git 브랜치 전략으로 병렬 기능 개발 + 안전한 통합
- 실제 동작 중 발생한 8가지 이상의 버그를 하나씩 커밋으로 해결

---

## 부록 — 사진/영상 체크리스트

| 슬라이드 | 필요 자료 | 준비 방법 |
|---------|---------|---------|
| 1 | 로봇 + 트레이 전경 | 실제 촬영 |
| 3 | 시약통 트레이 배치 | 실제 촬영 |
| 6 | rqt_graph 스크린샷 | `ros2 run rqt_graph rqt_graph` |
| 8, 9 | YOLO 추론 결과 오버레이 | `onnx_test_results/predict/` 폴더 이미지 사용 |
| 10 | cup/height bbox 설명 이미지 | 위 이미지에 화살표 추가 |
| 12, 13, 14 | 로봇 동작 시퀀스 영상 | 실제 동작 녹화 후 캡처 |
| 17 | 그리퍼 동작 | 실제 촬영 |
| 19 | HMI 탭별 스크린샷 | HMI 실행 후 각 탭 캡처 |
| 21 | 시연 영상 | 전체 시나리오 녹화 |
| 22 | 결과 비교 before/after | 실제 촬영 |

---

*이 문서는 Markdown 형식입니다. VS Code + Markdown PDF 익스텐션 또는 Pandoc으로 PDF 변환하세요.*
