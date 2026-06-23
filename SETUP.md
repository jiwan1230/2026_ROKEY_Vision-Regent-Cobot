# Setup Guide

다른 사용자가 이 저장소를 clone해서 프로젝트 환경을 그대로 구성하기 위한 가이드.

## 1. Prerequisites

- Ubuntu 22.04 + **ROS2 Humble** ([설치 가이드](https://docs.ros.org/en/humble/Installation.html))
- Python 3.10 (ROS2 Humble 기본)
- `colcon` (`sudo apt install python3-colcon-common-extensions`)

## 2. Clone

```bash
git clone git@github.com:jiwan1230/2026_ROKEY_Vision-Regent-Cobot.git
cd 2026_ROKEY_Vision-Regent-Cobot
```

## 3. Install Python dependencies

```bash
pip3 install -r requirements.txt
```

`ultralytics`, `opencv-python`, `numpy`, `Pillow`가 설치된다. (`rclpy`, `std_msgs`, `sensor_msgs` 등 ROS2 패키지는 ROS2 Humble 설치 시 이미 포함되어 있다.)

> **numpy2 / matplotlib 충돌 주의**: `numpy>=2`와 apt로 설치된 구버전 `matplotlib`(또는 `cv_bridge`)이 같이 있으면 `ultralytics` import 시 세그폴트가 날 수 있다. 증상이 나타나면:
> ```bash
> pip3 install --user --upgrade matplotlib
> ```
> 이 저장소의 `vision_pkg`는 애초에 이 문제를 피하려고 `cv_bridge`를 쓰지 않고 `vision_pkg/ros_image_utils.py`에서 numpy로 직접 이미지를 변환한다.

## 4. Build

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 5. Run (zero-config demo)

추가 설정 없이 바로 실행 가능 - `vision_pkg`에 번들된 YOLOv8n weight(`src/vision_pkg/weights/reagent_yolov8n.pt`, 3-class: cup/height/hand)와 샘플 이미지(`src/vision_pkg/sample_images/`, hand 없는 10장)를 사용해 카메라 없이도 전체 파이프라인을 시연한다.

```bash
ros2 launch launch/system_launch.py
```

HMI 창이 뜨고, 시약통 state/높이, 로봇 상태, 안전 패널이 실시간으로 갱신되는 것을 확인할 수 있다. HMI 없이 콘솔만 보려면:

```bash
ros2 launch launch/system_launch.py launch_hmi:=false
```

## 6. 실제 카메라/모델로 전환하기

| 용도 | launch argument |
|---|---|
| 실제 웹캠/USB 카메라 사용 | `source_mode:=device camera_index:=0` |
| 녹화된 영상 파일 사용 | `source_mode:=video_file` (`video_path`는 `src/vision_pkg/config/vision_params.yaml`에서 설정) |
| 직접 학습한 YOLO weight 사용 | `model_path:=/path/to/your.pt` |
| 다른 테스트 이미지 디렉토리 사용 | `image_dir:=/path/to/images` |

예시:

```bash
ros2 launch launch/system_launch.py source_mode:=device camera_index:=0 model_path:=/home/me/my_model.pt
```

세부 임계값(`target_height`, `tolerance`, `refill_min`, `overflow_max`, `confidence_threshold`)과 로봇 pose 좌표(`tube_0_approach_pose` 등)는 각각 `src/vision_pkg/config/vision_params.yaml`, `src/robot_control_pkg/config/robot_params.yaml`에서 직접 수정한다.

## 7. 패키지만 따로 실행하고 싶을 때

```bash
ros2 launch vision_pkg vision_launch.py            # 비전 파이프라인만
ros2 launch robot_control_pkg robot_control_launch.py  # 로봇 제어만 (mock)
ros2 run hmi_pkg hmi_node                          # HMI만
```

## 8. 실제 두산 M0609 로봇과 연동하려면

현재 `robot_control_pkg`의 `doosan_robot_control_node`/`gripper_control_node`는 mock이다(이동/그리퍼 동작을 sleep+log로 시뮬레이션). 실제 로봇 연동 시:

1. [doosan-robotics/doosan-robot2](https://github.com/doosan-robotics/doosan-robot2) ROS2 패키지를 워크스페이스에 추가한다.
2. `doosan_robot_control_node.py`의 `_simulate_move()`를 `DSR_ROBOT2.movel(...)` + `mwait()` 호출로 교체한다 (서비스 인터페이스 `/robot/move_to_pose`는 그대로 유지).
3. `gripper_control_node.py`의 `_simulate_grip()`을 실제 그리퍼 API(예: OnRobot RG `open_gripper()`/`close_gripper()`)로 교체한다.
4. `config/robot_params.yaml`의 placeholder 좌표를 실제 teaching한 좌표로 교체한다.

코드 구조와 명명은 이 프로젝트 작성자의 다른 Doosan bootcamp 코드(`pick_and_place_text/robot_move.py`, `onrobot.py`)와 동일한 스타일로 맞춰져 있어 교체가 비교적 직관적이다.

## Troubleshooting

| 증상 | 원인/해결 |
|---|---|
| `RuntimeError: model_path parameter is required` | `ros2 run`으로 노드를 직접 실행할 때는 launch 파일의 기본값이 적용되지 않는다. `--ros-args -p model_path:=<weight 경로>`를 추가하거나 launch 파일을 사용한다 |
| YOLO import 시 `AttributeError: _ARRAY_API not found` 또는 세그폴트 | 위 3번 항목의 numpy2/matplotlib 충돌. `pip3 install --user --upgrade matplotlib` |
| 카메라가 안 열림 (`source_mode:=device`) | `ls /dev/video*`로 장치 확인, `camera_index` 값을 실제 장치 번호로 변경 |
| HMI 창이 안 뜸 | `DISPLAY` 환경변수 확인 (SSH 접속 시 `-X`/`-Y` 옵션으로 X11 forwarding 필요), 또는 `launch_hmi:=false`로 HMI 없이 콘솔만 사용 |
