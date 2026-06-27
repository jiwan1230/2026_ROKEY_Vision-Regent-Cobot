# Setup Guide

이 저장소를 clone해서 프로젝트 환경을 구성하는 가이드.

---

## 1. 사전 요구사항

| 항목 | 버전 / 설치 방법 |
|---|---|
| OS | Ubuntu 22.04 |
| ROS2 | Humble ([설치 가이드](https://docs.ros.org/en/humble/Installation.html)) |
| Python | 3.10 (ROS2 Humble 기본 포함) |
| colcon | `sudo apt install python3-colcon-common-extensions` |
| PyQt5 | `sudo apt install python3-pyqt5` |
| DSR_MSGS2 | 실제 로봇 연동 시 필요 (아래 9번 항목 참고) |

---

## 2. Clone

```bash
git clone git@github.com:jiwan1230/2026_ROKEY_Vision-Regent-Cobot.git
cd 2026_ROKEY_Vision-Regent-Cobot
```

---

## 3. Python 의존성 설치

```bash
pip3 install -r requirements.txt
```

설치 패키지: `ultralytics`, `opencv-python`, `numpy`, `pymodbus`

> **numpy 2.x / cv_bridge 충돌 주의**  
> apt로 설치된 `cv_bridge`가 pip numpy 2.x와 ABI 불일치로 세그폴트를 일으킨다.  
> 이 프로젝트는 `cv_bridge`를 사용하지 않고 `vision_pkg/ros_image_utils.py`에서 직접 변환한다.  
> 증상이 나타나면:
> ```bash
> pip3 install --user --upgrade matplotlib
> ```

---

## 4. 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

빌드 후 `install/setup.bash`는 새 터미널마다 실행해야 한다.  
편의를 위해 `.bashrc`에 추가:

```bash
echo "source ~/Vision-Reagent-Cobot/install/setup.bash" >> ~/.bashrc
```

---

## 5. Zero-config 데모 실행 (카메라 없이)

`vision_pkg`에 YOLOv8n weight와 샘플 이미지가 번들되어 있어 추가 설정 없이 전체 파이프라인을 시연할 수 있다.

```bash
# 전체 시스템 실행 (Vision + Robot Control + HMI)
ros2 launch launch/system_launch.py

# HMI 없이 콘솔만 확인
ros2 launch launch/system_launch.py launch_hmi:=false
```

HMI가 뜨고 튜브 상태, 로봇 상태, 로그가 실시간으로 갱신된다.

---

## 6. 실제 카메라로 전환

```bash
# USB/웹캠 사용
ros2 launch launch/system_launch.py source_mode:=device camera_index:=0

# 녹화 영상 파일 사용
ros2 launch launch/system_launch.py source_mode:=video_file
# (video_path는 src/vision_pkg/config/vision_params.yaml에서 설정)

# 직접 학습한 YOLO weight 사용
ros2 launch launch/system_launch.py model_path:=/path/to/your.pt

# 다른 테스트 이미지 디렉토리 사용
ros2 launch launch/system_launch.py source_mode:=image_dir image_dir:=/path/to/images
```

---

## 7. 파라미터 설정

### Vision 파라미터 (`src/vision_pkg/config/vision_params.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `confidence_threshold` | 0.35 | YOLO 검출 신뢰도 임계값 |
| `publish_rate_hz` | 7.0 | 카메라 발행 주기 |
| `roi_x_max_px` | 540.0 | 오른쪽 노이즈 차단 ROI (hand 감지 예외) |
| `height_buffer_size` | 5 | 높이 안정화 median 필터 프레임 수 |
| `hand_detect_consecutive_frames` | 3 | 손 감지 확정 연속 프레임 수 |

### 로봇 파라미터 (`src/robot_control_pkg/config/robot_params.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `force_threshold` | 20.0 | 외력 감지 임계값 (N, TCP Cartesian) |
| `m_velocity` / `m_acceleration` | 60.0 | 이동 속도 / 가속도 |
| `normal_confirm_count` | 5 | 보충 완료 확인 연속 NORMAL 프레임 수 |
| `max_pour_attempts` | 20 | 보충 루프 안전 상한 |
| `poses.*` | 실측 좌표 | 모든 로봇 포즈 좌표 (mm, deg ZYZ euler) |

포즈 파라미터는 **HMI 관리자 탭**에서 런타임 수정 및 적용도 가능하다 (범위 검증 포함).

---

## 8. 패키지 개별 실행

```bash
# Vision 파이프라인만
ros2 launch vision_pkg vision_launch.py

# 로봇 제어만
ros2 launch robot_control_pkg robot_control_launch.py

# HMI만
ros2 run hmi_pkg hmi_node

# 외력 모니터링 (별도 터미널)
ros2 topic echo /robot/force_norm
ros2 topic echo /robot/force_detected
```

---

## 9. 실제 Doosan M0609 로봇 연동

현재 `doosan_robot_control_node`는 DSR API를 직접 사용하도록 구현되어 있다.

### 사전 준비

1. [doosan-robotics/doosan-robot2](https://github.com/doosan-robotics/doosan-robot2) ROS2 패키지를 워크스페이스에 추가
2. `dsr_msgs2` 패키지 빌드 완료 확인

### 연동 시 확인 사항

| 항목 | 내용 |
|---|---|
| 로봇 IP | 컨트롤러 IP 확인 후 launch 파일 수정 |
| 그리퍼 Modbus IP | `gripper_control_node` 내 `GRIPPER_IP` 설정 |
| 포즈 좌표 | `robot_params.yaml`의 placeholder를 실제 티칭 좌표로 교체 |
| 페이로드 설정 | 컨트롤러에 그리퍼+시약통 페이로드 등록 (외력 감지 정확도에 영향) |
| force_threshold | 실제 동작 환경에서 `ros2 topic echo /robot/force_norm`으로 정상값 확인 후 조정 |

---

## 10. Troubleshooting

| 증상 | 원인 / 해결 |
|---|---|
| `RuntimeError: model_path parameter is required` | launch 파일이 아닌 `ros2 run`으로 직접 실행. launch 파일 사용 또는 `--ros-args -p model_path:=<경로>` 추가 |
| YOLO import 시 세그폴트 / `_ARRAY_API not found` | numpy 2.x / cv_bridge ABI 충돌. `pip3 install --user --upgrade matplotlib` |
| 카메라가 안 열림 | `ls /dev/video*`로 장치 확인, `camera_index` 값 수정 |
| HMI 창이 안 뜸 | `DISPLAY` 환경변수 확인. SSH 접속 시 `-X` / `-Y` 옵션 필요. 또는 `launch_hmi:=false` |
| `force_norm` 토픽이 안 들어옴 | DSR 서비스 `aux_control/get_tool_force` 응답 확인. 로봇 컨트롤러 연결 상태 점검 |
| 그리퍼가 완료 전에 다음 동작으로 넘어감 | Modbus TCP 연결 확인. `gripper_control_node` 로그에서 완료 신호 수신 여부 확인 |
| 손 감지로 계속 정지 | `vision_params.yaml`의 `hand_detect_consecutive_frames` 값 높이기 (기본값 3). 또는 HMI 모션 제어 탭에서 손 감지 안전 토글 OFF |
| HMI 관리자 탭 포즈 Apply 실패 | 포즈 값이 DR-M0609 가동 범위(X/Y ±950mm, Z -200~1300mm) 초과 여부 확인 |
