# =============================================================================
# hmi_node.py  —  Vision-Regent-Cobot 통합 HMI 노드
# =============================================================================
# 변경 이력 (Change Log)
# -----------------------------------------------------------------------------
# 2026-06-24  soo  초기 마이그레이션
#                   - B-2_project → B-2_project_git (hmi_part 브랜치) 이전
#                   - CompressedImage(JPEG) 구독으로 변경 (QoS BEST_EFFORT 맞춤)
#                   - 외부 QTabWidget 래퍼 제거, uic.loadUi(self) 직접 로드
#                   - .ui 좌표 설정 위젯에 ROS 파라미터 초기값 채우기
#
# 2026-06-24  soo  시스템 관리자 탭 추가
#                   - QStackedWidget(page_login / page_admin) 기반 로그인 구현
#                   - 관리자 인증(admin/admin), 로그인·로그아웃 콜백
#                   - Vision 파라미터 설정 탭(발행속도, JPEG 품질, 카메라 인덱스)
#                   - YAML 좌표 설정 탭을 관리자 전용으로 이동
#
# 2026-06-24  soo  로봇 운전 상태 LED 인디케이터 추가
#                   - STOP(빨강) / RUN(초록) / ERROR(노랑) 3색 LED QLabel
#                   - robot_status 메시지 파싱 → LED·텍스트 색상 실시간 갱신
#
# 2026-06-24  soo  UI 언어 정리
#                   - 대시보드 전체 영어로 통일, 이모지 제거
#                   - 관리자·로그인 관련 텍스트는 한국어 유지
#
# 2026-06-25  soo  JOG 탭 추가
#                   - vision_camera_state.ui에 tab_jog (JOG Control) 탭 신규 생성
#                   - Joint Mode: J1~J6 각도 조그 (+/- 버튼, 스텝 콤보박스)
#                   - TCP Mode: X/Y/Z(mm) · A/B/C(deg) 위치 조그
#                   - JOG Speed 슬라이더(1~100%), STOP 버튼
#                   - 메인 QTabWidget 이름 tabWidget → JOG 변경에 따른 참조 수정
#
# 2026-06-27  soo  로봇 파라미터 적용 안전성 개선
#                   - 파라미터 범위 검증 추가 (_validate_params): 범위 초과 시 적용 차단
#                   - 적용 완료/실패 팝업 추가 (_on_apply_complete): 서비스 응답 대기 후 표시
#                   - _apply_params_thread에 threading.Event 도입 — 서비스 응답 후 팝업 발행
#                   - _DOOSAN_SCALAR_RANGES / _TASK_SCALAR_RANGES 상수 추가
#
# 2026-06-27  soo  비전 파라미터 confidence 연결
#                   - liquid_height_detector_node / tube_state_publisher_node 서비스 클라이언트 추가
#                   - tab_motion_vision: 바운딩박스 신뢰도(spin_vision_confidence) +
#                     높이 신뢰도(spin_vision_height_conf) 동적 추가
#                   - 로그인 시 GetParameters로 초기값 로드 (_load_vision_params)
#                   - btn_apply_vision_admin → SetParameters 두 노드에 동시 적용
#
# 2026-06-27  soo  좌표 자동계산 로직 개선 + waste_rotate 포즈 추가
#                   - _CALC_PARAMS 4개 → 9개: work_z_offset, pour_tube_offset,
#                     pour_z_offset, pour_ry_offset, grip_z_offset 추가
#                   - _recalc_derived_poses 재작성: approach 1개 기준 → 45개 전체 파생
#                     (기존: approach/work/pour 각각 독립 base / 신규: approach→work/pour/grip 오프셋)
#                   - base_pose_keys를 approach 2개만으로 축소
#                   - _all_pose_names()에 waste_rotate_* 4개 추가
# =============================================================================

import sys
import os
import threading
from datetime import datetime
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
# 2026-06-24 soo: side_image CompressedImage 구독으로 변경 (publisher QoS 맞춤)
from sensor_msgs.msg import CompressedImage
from ament_index_python.packages import get_package_share_directory

from PyQt5 import uic
from PyQt5.QtWidgets import (
    QApplication, QDialog, QMessageBox, QPushButton,
    QLineEdit, QLabel, QGroupBox, QGridLayout, QVBoxLayout, QWidget,
    QDoubleSpinBox, QSpinBox, QFormLayout, QRadioButton, QButtonGroup, QHBoxLayout,
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap

from std_srvs.srv import SetBool, Trigger
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from interfaces.msg import RobotStatus, TubeHeight, TubeState
from interfaces.srv import RequestRecheck, StopTask

STATE_LABELS = {
    TubeState.STATE_REFILL_NEEDED: "REFILL NEEDED",
    TubeState.STATE_NORMAL: "NORMAL",
    TubeState.STATE_DISPOSE_NEEDED: "DISPOSE NEEDED",
    TubeState.STATE_UNKNOWN: "UNKNOWN",
}
STATE_COLORS = {
    TubeState.STATE_REFILL_NEEDED: "#FBC02D",
    TubeState.STATE_NORMAL: "#2E7D32",
    TubeState.STATE_DISPOSE_NEEDED: "#c0392b",
    TubeState.STATE_UNKNOWN: "#888888",
}

RESOLUTION_OPTIONS = ["640x480", "1280x720", "1920x1080"]


def _all_pose_names(num_tubes: int = 3, num_trays: int = 3) -> list:
    """robot_control_pkg.poses.all_pose_names 와 동일한 목록. 패키지 의존 없이 인라인."""
    names = [
        "home_pose",
        "waste_approach_pose",
        "waste_release_pose",
        "waste_rotate_pose",
        "waste_rotate_middle_pose",
        "waste_rotate_reagent_pose",
        "waste_rotate_tube_pose",
        "tray_tool_stand_approach_pose",
        "tray_tool_stand_grip_pose",
        "refill_source_approach_pose",
        "refill_source_grip_pose",
    ]
    for t in range(num_trays):
        names += [
            f"tray_transfer_{t}_tool_approach_pose",
            f"tray_transfer_{t}_tool_preinsert_pose",
            f"tray_transfer_{t}_tool_insert_pose",
            f"tray_transfer_{t}_lift_pose",
            f"tray_transfer_{t}_success_approach_pose",
            f"tray_transfer_{t}_success_place_pose",
            f"tray_transfer_{t}_tool_detach_pose",
        ]
        for u in range(num_tubes):
            names += [
                f"refill_target_{t}_{u}_approach_pose",
                f"refill_target_{t}_{u}_work_pose",
                f"refill_target_{t}_{u}_pour_pose",
                f"dispose_tray_{t}_tube_{u}_approach_pose",
                f"dispose_tray_{t}_tube_{u}_grip_pose",
            ]
    return names

# doosan_robot_control_node에 declare된 스칼라 파라미터 (SetParameters로 전송)
_DOOSAN_SCALAR = [
    ('m_velocity',        float),
    ('m_acceleration',    float),
    ('d_velocity',        float),
    ('d_acceleration',    float),
    ('move_duration_sec', float),
]
# UI 전용 계산 파라미터 - YAML 참조용이며 로봇 노드에 declare 안 됨 (전송하지 않음)
_CALC_PARAMS = [
    ('tube_gap',          70.0),
    ('tray_gap',          100.0),
    ('tube_crood',        1),
    ('tray_crood',        0),
    ('work_z_offset',    -105.0),   # approach → work Z 차이
    ('pour_tube_offset',  35.0),    # approach → pour 튜브축 방향 차이
    ('pour_z_offset',    -45.0),    # approach → pour Z 차이
    ('pour_ry_offset',   -45.0),    # approach → pour RY 차이
    ('grip_z_offset',   -120.0),    # approach → grip Z 차이 (dispose)
]
# robot_task_manager_node에 declare된 스칼라 파라미터
_TASK_SCALAR = [
    ('grip_retry_count',         int),
    ('recheck_timeout_sec',      float),
    ('service_call_timeout_sec', float),
    ('max_pour_attempts',        int),
    ('normal_confirm_count',     int),
    ('num_trays',                int),
]

# 파라미터 허용 범위 (min, max)
_DOOSAN_SCALAR_RANGES = {
    'm_velocity':        (1.0,  100.0),
    'm_acceleration':    (1.0,  100.0),
    'd_velocity':        (1.0,  100.0),
    'd_acceleration':    (1.0,  100.0),
    'move_duration_sec': (0.05, 10.0),
}
_TASK_SCALAR_RANGES = {
    'grip_retry_count':         (0,    20),
    'recheck_timeout_sec':      (0.5,  300.0),
    'service_call_timeout_sec': (1.0,  600.0),
    'max_pour_attempts':        (1,    100),
    'normal_confirm_count':     (1,    50),
    'num_trays':                (1,    5),
}


# 2026-06-24 soo: CompressedImage(JPEG) → bgr8 numpy 변환
def decode_compressed(msg) -> np.ndarray:
    return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)


# raw sensor_msgs/Image(bgr8) → bgr8 numpy 변환 (yolo_image용)
def decode_raw(msg) -> np.ndarray:
    return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)


# =====================================================================
# 1. ROS 2 HMI 통합 노드
# =====================================================================
class IntegratedHMINode(Node):
    def __init__(self):
        super().__init__("hmi_node")

        self.latest_frame = None
        self.latest_yolo_frame = None
        self.latest_tube_state = None
        self.latest_tube_height = None
        self.latest_robot_status = None
        self.camera_ok = True
        self.hand_detected = False
        self.gripper_closed = False
        self.grip_failed = False
        self.system_running = False
        # vision_pkg가 /vision/log로 보내는 사람이 읽을 이벤트 문장들. 새로 들어오는
        # 만큼만 _refresh_dashboard에서 꺼내가도록 리스트로 쌓아두기만 함 (Qt 위젯은
        # ROS 스핀 스레드가 아니라 GUI 스레드에서만 만져야 해서 여기서 직접 안 그림).
        self.vision_log_messages = []

        self.declare_parameter('velocity', 60.0)
        self.declare_parameter('acceleration', 60.0)
        # 2026-06-24 soo: 관리자 탭 YAML 좌표 설정용 ROS 파라미터 선언
        self.declare_parameter('poses.home_pose', [400.0, 0.0, 400.0, 0.0, 180.0, 0.0])
        self.declare_parameter('poses.waste_pose', [500.0, -350.0, 200.0, 0.0, 180.0, 0.0])
        self.declare_parameter('poses.normal_tray_approach_pose', [500.0, 350.0, 300.0, 0.0, 180.0, 0.0])
        self.declare_parameter('poses.tube_0_approach_pose', [300.0, -150.0, 300.0, 0.0, 180.0, 0.0])

        from sensor_msgs.msg import Image
        # 2026-06-24 soo: side_image publisher(side_camera_node)와 QoS 맞춤 - BEST_EFFORT/depth=1
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(CompressedImage, "/vision/side_image", self._on_image, image_qos)
        self.create_subscription(Image, "/vision/yolo_image", self._on_yolo_image, 10)

        self.create_subscription(TubeState, "/vision/tube_state", self._on_tube_state, 10)
        self.create_subscription(TubeHeight, "/vision/tube_height", self._on_tube_height, 10)
        self.create_subscription(RobotStatus, "/robot/status", self._on_robot_status, 10)
        self.create_subscription(Bool, "/vision/camera_status", self._on_camera_status, 10)
        self.create_subscription(Bool, "/vision/hand_detected", self._on_hand_detected, 10)
        self.create_subscription(Bool, '/gripper/is_closed', self._on_gripper_state, 10)
        self.create_subscription(Bool, '/gripper/grip_failed', self._on_grip_failed, 10)
        self.create_subscription(Bool, '/robot/system_running', self._on_system_running, 10)
        self.create_subscription(String, '/vision/log', self._on_vision_log, 10)

        self.stop_task_client = self.create_client(StopTask, "/robot/stop_task")
        self.recheck_client = self.create_client(RequestRecheck, "/vision/request_recheck")
        self.set_system_running_client = self.create_client(SetBool, "/robot/set_system_running")
        self.reset_slot_anchors_client = self.create_client(Trigger, "/vision/reset_slot_anchors")
        self.reset_handled_slots_client = self.create_client(Trigger, "/vision/reset_handled_slots")

        self.get_param_doosan_client = self.create_client(
            GetParameters, '/doosan_robot_control_node/get_parameters')
        self.set_param_doosan_client = self.create_client(
            SetParameters, '/doosan_robot_control_node/set_parameters')
        self.get_param_task_client = self.create_client(
            GetParameters, '/robot_task_manager_node/get_parameters')
        self.set_param_task_client = self.create_client(
            SetParameters, '/robot_task_manager_node/set_parameters')
        # 2026-06-27 soo: 비전 노드 파라미터 서비스 클라이언트 추가
        self.get_param_camera_client = self.create_client(
            GetParameters, '/side_camera_node/get_parameters')
        self.set_param_camera_client = self.create_client(
            SetParameters, '/side_camera_node/set_parameters')
        self.get_param_detector_client = self.create_client(
            GetParameters, '/liquid_height_detector_node/get_parameters')
        self.set_param_detector_client = self.create_client(
            SetParameters, '/liquid_height_detector_node/set_parameters')
        self.get_param_tube_state_client = self.create_client(
            GetParameters, '/tube_state_publisher_node/get_parameters')
        self.set_param_tube_state_client = self.create_client(
            SetParameters, '/tube_state_publisher_node/set_parameters')

        self.resolution_pub = self.create_publisher(String, "/camera/resolution_cmd", 10)
        self.yolo_enable_pub = self.create_publisher(Bool, "/vision/yolo_enabled", 10)

    def _on_image(self, msg):
        try:
            self.latest_frame = decode_compressed(msg)   # CompressedImage(JPEG)
        except Exception as e:
            self.get_logger().warn(f"Failed to decode image: {e}")

    def _on_yolo_image(self, msg):
        try:
            self.latest_yolo_frame = decode_raw(msg)     # raw Image(bgr8)
        except Exception as e:
            self.get_logger().warn(f"Failed to decode yolo image: {e}")

    def _on_tube_state(self, msg): self.latest_tube_state = msg
    def _on_tube_height(self, msg): self.latest_tube_height = msg
    def _on_robot_status(self, msg): self.latest_robot_status = msg
    def _on_camera_status(self, msg): self.camera_ok = msg.data
    def _on_hand_detected(self, msg): self.hand_detected = msg.data
    def _on_gripper_state(self, msg): self.gripper_closed = msg.data
    def _on_grip_failed(self, msg): self.grip_failed = msg.data
    def _on_system_running(self, msg): self.system_running = msg.data

    def _on_vision_log(self, msg):
        self.vision_log_messages.append(msg.data)

    def call_set_system_running(self, running: bool):
        return self.set_system_running_client.call_async(SetBool.Request(data=running))

    def call_stop_task(self, is_emergency=False):
        return self.stop_task_client.call_async(StopTask.Request(stop=True, is_emergency=is_emergency))
    def call_request_recheck(self): return self.recheck_client.call_async(RequestRecheck.Request(request=True))

    def call_reset_slot_anchors(self):
        return self.reset_slot_anchors_client.call_async(Trigger.Request())

    def call_reset_handled_slots(self):
        return self.reset_handled_slots_client.call_async(Trigger.Request())

    def publish_resolution(self, res_str: str):
        msg = String(); msg.data = res_str; self.resolution_pub.publish(msg)

    def publish_yolo_enabled(self, enabled: bool):
        msg = Bool(); msg.data = enabled; self.yolo_enable_pub.publish(msg)

    def call_get_param_doosan(self, names: list):
        req = GetParameters.Request()
        req.names = names
        return self.get_param_doosan_client.call_async(req)

    def call_set_param_doosan(self, params: list):
        req = SetParameters.Request()
        req.parameters = params
        return self.set_param_doosan_client.call_async(req)

    def call_get_param_task(self, names: list):
        req = GetParameters.Request()
        req.names = names
        return self.get_param_task_client.call_async(req)

    def call_set_param_task(self, params: list):
        req = SetParameters.Request()
        req.parameters = params
        return self.set_param_task_client.call_async(req)

    # 2026-06-27 soo: 비전 노드 파라미터 get/set 헬퍼
    def call_get_param_camera(self, names: list):
        req = GetParameters.Request()
        req.names = names
        return self.get_param_camera_client.call_async(req)

    def call_set_param_camera(self, params: list):
        req = SetParameters.Request()
        req.parameters = params
        return self.set_param_camera_client.call_async(req)

    def call_get_param_detector(self, names: list):
        req = GetParameters.Request()
        req.names = names
        return self.get_param_detector_client.call_async(req)

    def call_set_param_detector(self, params: list):
        req = SetParameters.Request()
        req.parameters = params
        return self.set_param_detector_client.call_async(req)

    def call_get_param_tube_state(self, names: list):
        req = GetParameters.Request()
        req.names = names
        return self.get_param_tube_state_client.call_async(req)

    def call_set_param_tube_state(self, params: list):
        req = SetParameters.Request()
        req.parameters = params
        return self.set_param_tube_state_client.call_async(req)


# =====================================================================
# 2. PyQt 기반 UI 화면 클래스 (.ui 파일의 탭 구조를 그대로 사용)
# 2026-06-24 soo: 외부 QTabWidget 제거 - uic.loadUi(self)로 직접 로드해 탭 중복 제거
# =====================================================================
class HMIDashboardApp(QDialog):
    # rclpy's future.add_done_callback() fires on the background ROS spin
    # thread, not the Qt GUI thread - touching QTextEdit from there directly
    # (the old self.log() call) crashes Qt. Routing through a signal lets Qt
    # marshal the string-only payload onto the GUI thread safely.
    log_signal               = pyqtSignal(str)
    _doosan_params_ready     = pyqtSignal(object, object)  # (names, pvalues)
    _task_params_ready       = pyqtSignal(object, object)  # (names, pvalues)
    _camera_params_ready          = pyqtSignal(object, object)  # 2026-06-27 soo: FPS
    _detector_roi_params_ready    = pyqtSignal(object, object)  # 2026-06-27 soo: ROI
    _detector_params_ready        = pyqtSignal(object, object)  # (names, pvalues)
    _tube_state_params_ready      = pyqtSignal(object, object)  # (names, pvalues)
    _detector_buffer_params_ready = pyqtSignal(object, object)  # 2026-06-27 soo: 버퍼 파라미터
    _apply_complete_signal   = pyqtSignal(bool, str)       # (success, message)

    def __init__(self, node: IntegratedHMINode):
        super().__init__()
        self.node = node
        self.log_signal.connect(self.log)
        self._doosan_params_ready.connect(self._apply_doosan_params)
        self._task_params_ready.connect(self._apply_task_params)
        self._camera_params_ready.connect(self._apply_camera_params)                # 2026-06-27 soo
        self._detector_roi_params_ready.connect(self._apply_detector_roi_params)   # 2026-06-27 soo
        self._detector_params_ready.connect(self._apply_detector_params)           # 2026-06-27 soo
        self._tube_state_params_ready.connect(self._apply_tube_state_params)       # 2026-06-27 soo
        self._detector_buffer_params_ready.connect(self._apply_detector_buffer_params)  # 2026-06-27 soo
        self._apply_complete_signal.connect(self._on_apply_complete)
        self.yolo_enabled = False
        self._grip_fail_shown = False
        self._vision_log_seen = 0
        # 2026-06-24 soo: 관리자 인증 상태 플래그 — 탭 전환 시 로그인 페이지 강제 표시에 사용
        self._admin_authenticated = False

        package_share_directory = get_package_share_directory('hmi_pkg')
        ui_path = os.path.join(package_share_directory, 'ui', 'vision_camera_state.ui')

        if not os.path.exists(ui_path):
            self.node.get_logger().error(f"UI 파일을 찾을 수 없습니다: {ui_path}")

        uic.loadUi(ui_path, self)  # 2026-06-24 soo: self.ui(QDialog) 래퍼 제거, 직접 로드

        # QPushButton의 autoDefault는 기본값이 True라서, 어느 탭/페이지에 있든 Enter가
        # 그 순간 보이는 버튼 하나를 임의로 클릭해버릴 수 있음 (예: 관리자 로그인 중
        # Enter -> 로그인 성공으로 막 보이게 된 로그아웃 버튼이 같은 키 이벤트에서
        # 클릭되어 바로 로그아웃되는 버그). Enter는 우리가 명시적으로 연결한 동작
        # (returnPressed)에만 반응하게, 모든 버튼의 default/autoDefault를 끈다.
        for btn in self.findChildren(QPushButton):
            btn.setAutoDefault(False)
            btn.setDefault(False)

        # ── 운영 대시보드 탭 버튼 연결 ──────────────────────────────
        self.pushButton.clicked.connect(self.on_start)
        self.pushButton_2.clicked.connect(self.on_stop)
        self.pushButton_3.clicked.connect(self.on_recheck)
        self.pushButton_4.clicked.connect(self.on_emergency_stop)
        self.pushButton_4.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.btn_yolo_toggle_2.clicked.connect(self.on_reset_slot_anchor)

        self.comboBox_resolution.addItems(RESOLUTION_OPTIONS)
        self.btn_apply_resolution.clicked.connect(self.on_apply_resolution)

        self.btn_yolo_toggle.setChecked(False)
        self.btn_yolo_toggle.clicked.connect(self.on_toggle_yolo)

        self.btn_clear_fail.clicked.connect(self.on_clear_grip_fail)

        # ── 2026-06-24 soo: 시스템 관리자 탭 연결 ───────────────────
        self.tabWidget.currentChanged.connect(self._on_tab_changed)
        self.btn_admin_login.clicked.connect(self._on_admin_login)
        self.btn_admin_logout.clicked.connect(self._on_admin_logout)
        self.input_admin_pw.returnPressed.connect(self._on_admin_login)
        self.btn_apply_robot_param.clicked.connect(self.on_apply_robot_param)

        self.tube_widgets = [
            (self.tube0_color_box, self.tube0_status_label, self.tube0_val_label),
            (self.tube1_color_box, self.tube1_status_label, self.tube1_val_label),
            (self.tube2_color_box, self.tube2_status_label, self.tube2_val_label),
        ]

        self._build_robot_param_ui()
        self._setup_vision_feature_toggles()  # 2026-06-27 soo: Vision 기능 ON/OFF 토글
        self._setup_vision_conf_ui()          # 2026-06-27 soo: 비전 confidence UI 초기화
        self._setup_vision_buffer_ui()        # 2026-06-27 soo: 추론 버퍼 UI 초기화
        self.log("PyQt HMI System initialized.")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh_dashboard)
        self.timer.start(150)

    # -----------------------------------------------------------------
    # 로그 출력
    # -----------------------------------------------------------------
    def log(self, message):
        ts = datetime.now().strftime("%H:%M:%S")
        self.textEdit.append(f"[{ts}] {message}")

    # -----------------------------------------------------------------
    # 운영 버튼 콜백
    # -----------------------------------------------------------------
    def on_start(self):
        future = self.node.call_set_system_running(True)
        self.log("Start pressed - enabling automatic vision-driven control")
        future.add_done_callback(lambda f: self._log_service_result("set_system_running", f))

    def on_stop(self):
        future = self.node.call_set_system_running(False)
        self.log("Stop pressed - disabling automatic vision-driven control")
        future.add_done_callback(lambda f: self._log_service_result("set_system_running", f))
        stop_future = self.node.call_stop_task()
        stop_future.add_done_callback(lambda f: self._log_service_result("stop_task", f))

    def on_recheck(self):
        future = self.node.call_request_recheck()
        self.log("Recheck requested")
        future.add_done_callback(lambda f: self._log_service_result("request_recheck", f))

    def on_reset_slot_anchor(self):
        # 처음 부트스트랩된 slot anchor가 잘못 잡혔을 때 수동으로 다시 잡게 하는 복구용
        # 버튼 - 새 트레이로 넘어갈 때 robot_task_manager_node가 자동으로 호출하는 것과
        # 동일한 두 서비스를 그대로 호출한다.
        anchors_future = self.node.call_reset_slot_anchors()
        self.log("Slot anchor reset requested")
        anchors_future.add_done_callback(lambda f: self._log_service_result("reset_slot_anchors", f))
        handled_future = self.node.call_reset_handled_slots()
        handled_future.add_done_callback(lambda f: self._log_service_result("reset_handled_slots", f))

    def on_emergency_stop(self):
        future = self.node.call_set_system_running(False)
        self.log("EMERGENCY STOP pressed")
        future.add_done_callback(lambda f: self._log_service_result("set_system_running", f))
        stop_future = self.node.call_stop_task(is_emergency=True)
        stop_future.add_done_callback(lambda f: self._log_service_result("stop_task", f))

    def _log_service_result(self, name, future):
        # Called from the ROS spin thread (future.add_done_callback) - emit
        # the signal rather than touching textEdit directly here.
        try:
            result = future.result()
            self.log_signal.emit(f"{name} -> success={result.success} message={result.message}")
        except Exception as e:
            self.log_signal.emit(f"{name} -> error: {e}")

    # -----------------------------------------------------------------
    # 해상도 변경 콜백
    # -----------------------------------------------------------------
    def on_apply_resolution(self):
        res_str = self.comboBox_resolution.currentText()
        self.node.publish_resolution(res_str)
        self.log(f"해상도 변경 요청: {res_str}")

    # -----------------------------------------------------------------
    # YOLO 추론 토글 콜백
    # 2026-06-24 soo: 버튼 텍스트 영어로 통일 (대시보드 언어 정리)
    # -----------------------------------------------------------------
    def on_toggle_yolo(self):
        self.yolo_enabled = self.btn_yolo_toggle.isChecked()
        self.node.publish_yolo_enabled(self.yolo_enabled)
        if self.yolo_enabled:
            self.btn_yolo_toggle.setText("YOLO Inference ON")
            self.btn_yolo_toggle.setStyleSheet("background-color:#1976D2; color:white; font-weight:bold;")
        else:
            self.btn_yolo_toggle.setText("YOLO Inference OFF")
            self.btn_yolo_toggle.setStyleSheet("")
        self.log(f"YOLO 추론 결과 표시: {'ON' if self.yolo_enabled else 'OFF'}")

    # -----------------------------------------------------------------
    # 그리퍼 강도 콜백
    # -----------------------------------------------------------------
    # -----------------------------------------------------------------
    # 그립 실패 알림 콜백
    # -----------------------------------------------------------------
    def on_clear_grip_fail(self):
        self.node.grip_failed = False
        self._grip_fail_shown = False
        self.lbl_grip_fail_alert.setVisible(False)
        self.btn_clear_fail.setVisible(False)
        self.log("그립 실패 알림 확인됨 (초기화)")

    # -----------------------------------------------------------------
    # 2026-06-24 soo: 시스템 관리자 탭 로그인/로그아웃
    #   - QStackedWidget(stack_admin): page 0=로그인, page 1=관리자 패널
    #   - 탭 전환 시 미인증이면 항상 로그인 페이지로 되돌림
    # -----------------------------------------------------------------
    def _on_tab_changed(self, index: int):
        admin_index = self.tabWidget.indexOf(self.tab_admin)
        if index == admin_index and not self._admin_authenticated:
            self.stack_admin.setCurrentIndex(0)
            self.input_admin_id.clear()
            self.input_admin_pw.clear()
            self.lbl_admin_login_status.setText("")

    def _on_admin_login(self):
        uid = self.input_admin_id.text().strip()
        pw = self.input_admin_pw.text()
        if uid == "admin" and pw == "admin":
            self._admin_authenticated = True
            self.val_si_login_time.setText(datetime.now().strftime("%H:%M:%S"))
            self.stack_admin.setCurrentIndex(1)
            self.lbl_admin_login_status.setText("")
            self.log("시스템 관리자 로그인 성공")
            self._load_robot_params()
            self._load_vision_params()  # 2026-06-27 soo: 로그인 시 비전 파라미터 초기값 로드
        else:
            self.lbl_admin_login_status.setText("아이디 또는 비밀번호가 올바르지 않습니다.")
            self.input_admin_pw.clear()
            self.log("시스템 관리자 로그인 실패 (잘못된 자격증명)")

    def _on_admin_logout(self):
        self._admin_authenticated = False
        self.stack_admin.setCurrentIndex(0)
        self.input_admin_id.clear()
        self.input_admin_pw.clear()
        self.lbl_admin_login_status.setText("")
        self.log("시스템 관리자 로그아웃")

    # -----------------------------------------------------------------
    # 2026-06-24 soo: 로봇 운전 상태 LED 인디케이터
    #   - STOP(빨강): 대기/정지 상태
    #   - RUN(초록): 동작 중
    #   - ERROR(노랑): 오류/실패
    #   - robot_status 토픽 수신 시 _refresh_dashboard()에서 호출
    # -----------------------------------------------------------------
    _LED_ON_STOP  = "background-color:#e74c3c; border-radius:14px; border:2px solid #c0392b;"
    _LED_ON_RUN   = "background-color:#2ecc71; border-radius:14px; border:2px solid #27ae60;"
    _LED_ON_ERROR = "background-color:#f39c12; border-radius:14px; border:2px solid #d68910;"
    _LED_OFF      = "background-color:#555555; border-radius:14px; border:2px solid #333333;"
    _TXT_STOP  = "font-size:13px; font-weight:bold; color:#e74c3c;"
    _TXT_RUN   = "font-size:13px; font-weight:bold; color:#2ecc71;"
    _TXT_ERROR = "font-size:13px; font-weight:bold; color:#f39c12;"
    _TXT_OFF   = "font-size:13px; font-weight:bold; color:#888888;"

    def _update_robot_led(self, status_msg):
        s = (status_msg.status.lower() if status_msg and status_msg.status else "")
        if "error" in s or "fail" in s or "오류" in s:
            stop, run, err = self._LED_OFF, self._LED_OFF, self._LED_ON_ERROR
            ts, tr, te = self._TXT_OFF, self._TXT_OFF, self._TXT_ERROR
        elif s and "idle" not in s and "stop" not in s and "정지" not in s:
            stop, run, err = self._LED_OFF, self._LED_ON_RUN, self._LED_OFF
            ts, tr, te = self._TXT_OFF, self._TXT_RUN, self._TXT_OFF
        else:
            stop, run, err = self._LED_ON_STOP, self._LED_OFF, self._LED_OFF
            ts, tr, te = self._TXT_STOP, self._TXT_OFF, self._TXT_OFF
        self.lbl_led_stop.setStyleSheet(stop)
        self.lbl_led_run.setStyleSheet(run)
        self.lbl_led_error.setStyleSheet(err)
        self.lbl_led_stop_text.setStyleSheet(ts)
        self.lbl_led_run_text.setStyleSheet(tr)
        self.lbl_led_error_text.setStyleSheet(te)

    def _update_grip_fail_banner(self):
        if self.node.grip_failed and not self._grip_fail_shown:
            self._grip_fail_shown = True
            self.lbl_grip_fail_alert.setVisible(True)
            self.btn_clear_fail.setVisible(True)
            self.log("⚠ 그립 실패 감지됨")

    def _update_vision_log(self):
        messages = self.node.vision_log_messages
        if self._vision_log_seen >= len(messages):
            return
        ts = datetime.now().strftime("%H:%M:%S")
        for text in messages[self._vision_log_seen:]:
            self.textEdit_vision_log.append(f"[{ts}] {text}")
        self._vision_log_seen = len(messages)

    # -----------------------------------------------------------------
    # 화면 실시간 폴링 (150ms)
    # -----------------------------------------------------------------
    def _refresh_dashboard(self):
        if self.yolo_enabled and self.node.latest_yolo_frame is not None:
            frame = self.node.latest_yolo_frame
        else:
            frame = self.node.latest_frame

        if frame is not None:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            # PyQt5: QImage은 bytes 포인터만 참조 → GC 전에 .copy()로 Qt 내부 복사 강제
            img_bytes = rgb_image.tobytes()
            qt_img = QImage(img_bytes, w, h, ch * w, QImage.Format_RGB888).copy()
            lw, lh = self.videoLabel.width(), self.videoLabel.height()
            if lw > 0 and lh > 0:
                qt_img = qt_img.scaled(lw, lh, aspectRatioMode=1)
            self.videoLabel.setPixmap(QPixmap.fromImage(qt_img))

        state_msg = self.node.latest_tube_state
        height_msg = self.node.latest_tube_height

        heights = {}
        if height_msg is not None:
            heights = dict(zip(height_msg.tube_index, height_msg.liquid_height))

        if state_msg is not None:
            for idx, state in zip(state_msg.tube_index, state_msg.state):
                if idx >= len(self.tube_widgets):
                    continue
                color_box, state_text, height_text = self.tube_widgets[idx]
                color_box.setStyleSheet(f"background-color: {STATE_COLORS.get(state, '#888888')};")
                state_text.setText(STATE_LABELS.get(state, str(state)))
                if idx in heights:
                    height_text.setText(f"{heights[idx]:.1f} mm")

        status_msg = self.node.latest_robot_status
        if status_msg is not None:
            self.label_tube0_11.setText(f"{status_msg.status}")
            self.label_tube0_10.setText(f"{status_msg.current_task or '--'}")
            self.label_tube0_9.setText(f"{status_msg.detail or '--'}")
            self.log(status_msg.log)
        self._update_robot_led(status_msg)

        self.label_tube0_7.setText("OK" if self.node.camera_ok else "DISCONNECTED")

        if self.node.hand_detected:
            self.label_tube0_8.setText("YES - motion withheld")
            self.label_tube0_8.setStyleSheet("color: red; font-weight: bold;")
        else:
            self.label_tube0_8.setText("Clear")
            self.label_tube0_8.setStyleSheet("color: white;")

        self._update_grip_fail_banner()
        self._update_vision_log()

    # -----------------------------------------------------------------
    # 로봇 파라미터 탭 UI 동적 빌드
    # .ui의 정적 QLabel 위젯들을 비우고 실제 파라미터 이름 기준 QLineEdit으로 재구성
    # -----------------------------------------------------------------
    def _build_robot_param_ui(self):
        container = self.scrollAreaWidgetContents_robot_param
        layout = container.layout()

        # 기존 위젯 전부 제거 (btn_apply_robot_param은 마지막에 다시 추가)
        while layout.count():
            item = layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.setParent(None)
                if w is not self.btn_apply_robot_param:
                    w.deleteLater()

        edit_style = (
            "background:#1a1a2e; color:#00e5ff; font-size:13px;"
            " font-family:monospace; border:1px solid #1565C0;"
            " border-radius:4px; padding:3px 8px;"
        )
        lbl_style = "font-size:13px; font-weight:bold;"
        hdr_style = (
            "font-size:12px; font-weight:bold;"
            " background:#E3F2FD; padding:3px 6px;"
        )
        apply_btn_style = (
            "background:#1565C0; color:white; font-size:13px; font-weight:bold;"
            " border-radius:4px; padding:5px 12px; margin-top:4px;"
        )

        # ── Doosan 스칼라 파라미터 그룹 ──────────────────────────────
        grp_doosan = QGroupBox("Doosan 파라미터 (doosan_robot_control_node)")
        grid_d = QGridLayout(grp_doosan)
        grid_d.setHorizontalSpacing(12)
        grid_d.setVerticalSpacing(8)
        self._scalar_edits = {'doosan': {}, 'task': {}}

        for row, (name, _) in enumerate(_DOOSAN_SCALAR):
            lbl = QLabel(f"{name}:")
            lbl.setStyleSheet(lbl_style)
            edit = QLineEdit("0.0")
            edit.setStyleSheet(edit_style)
            edit.setMinimumWidth(110)
            grid_d.addWidget(lbl, row, 0)
            grid_d.addWidget(edit, row, 1)
            self._scalar_edits['doosan'][name] = edit
        btn_d = QPushButton("Doosan 파라미터 적용")
        btn_d.setStyleSheet(apply_btn_style)
        btn_d.clicked.connect(self._on_apply_doosan_scalar)
        grid_d.addWidget(btn_d, len(_DOOSAN_SCALAR), 0, 1, 2)
        layout.addWidget(grp_doosan)

        # ── 자동 재계산용 gap 파라미터 그룹 (UI 전용, 로봇에 전송 안 함) ──
        grp_calc = QGroupBox("좌표 자동 계산 파라미터 (UI 전용 — 로봇에 전송되지 않음)")
        grid_c = QGridLayout(grp_calc)
        grid_c.setHorizontalSpacing(12)
        grid_c.setVerticalSpacing(8)
        self._calc_edits = {}

        for row, (name, default) in enumerate(_CALC_PARAMS):
            lbl = QLabel(f"{name}:")
            lbl.setStyleSheet(lbl_style)
            edit = QLineEdit(str(default))
            edit.setStyleSheet(edit_style)
            edit.setMinimumWidth(110)
            edit.editingFinished.connect(self._on_gap_param_changed)
            grid_c.addWidget(lbl, row, 0)
            grid_c.addWidget(edit, row, 1)
            self._calc_edits[name] = edit

        note = QLabel(
            "※ tube_gap/tray_gap 변경 시 refill_target / dispose_tray 포즈가 자동 재계산됩니다.\n"
            "   기준 포즈: refill_target_0_0_* 와 dispose_tray_0_tube_0_* (tray/tube 인덱스 0)"
        )
        note.setStyleSheet("font-size:11px; color:#aaaaaa; padding:4px 0;")
        note.setWordWrap(True)
        grid_c.addWidget(note, len(_CALC_PARAMS), 0, 1, 2)
        layout.addWidget(grp_calc)

        # ── Task Manager 스칼라 파라미터 그룹 ────────────────────────
        grp_task = QGroupBox("Task Manager 파라미터 (robot_task_manager_node)")
        grid_t = QGridLayout(grp_task)
        grid_t.setHorizontalSpacing(12)
        grid_t.setVerticalSpacing(8)

        for row, (name, _) in enumerate(_TASK_SCALAR):
            lbl = QLabel(f"{name}:")
            lbl.setStyleSheet(lbl_style)
            edit = QLineEdit("0")
            edit.setStyleSheet(edit_style)
            edit.setMinimumWidth(110)
            grid_t.addWidget(lbl, row, 0)
            grid_t.addWidget(edit, row, 1)
            self._scalar_edits['task'][name] = edit
        btn_t = QPushButton("Task 파라미터 적용")
        btn_t.setStyleSheet(apply_btn_style)
        btn_t.clicked.connect(self._on_apply_task_scalar)
        grid_t.addWidget(btn_t, len(_TASK_SCALAR), 0, 1, 2)
        layout.addWidget(grp_task)

        # ── 포즈 파라미터 그룹 (poses.*) ────────────────────────────
        grp_poses = QGroupBox("포즈 설정 (poses.*)")
        grid_p = QGridLayout(grp_poses)
        grid_p.setHorizontalSpacing(4)
        grid_p.setVerticalSpacing(3)

        for col, hdr in enumerate(['포즈 이름', 'X(mm)', 'Y(mm)', 'Z(mm)', 'RX(°)', 'RY(°)', 'RZ(°)']):
            h = QLabel(hdr)
            h.setStyleSheet(hdr_style)
            h.setAlignment(Qt.AlignCenter)
            grid_p.addWidget(h, 0, col)

        self._pose_edits = {}
        pose_edit_style = (
            "background:#1a1a2e; color:#00e5ff; font-size:12px;"
            " font-family:monospace; border:1px solid #1565C0;"
            " border-radius:3px; padding:2px 5px;"
        )
        # 자동 재계산 기준 포즈 — approach만 사용자가 직접 입력, 나머지는 파생
        base_pose_keys = {
            'refill_target_0_0_approach_pose',
            'dispose_tray_0_tube_0_approach_pose',
        }

        for row_i, pose_name in enumerate(_all_pose_names(num_tubes=3, num_trays=3), start=1):
            lbl = QLabel(f"{pose_name}:")
            lbl.setStyleSheet("font-size:11px; font-weight:bold;")
            grid_p.addWidget(lbl, row_i, 0)

            edits = []
            for col_i in range(6):
                e = QLineEdit("0.0")
                e.setStyleSheet(pose_edit_style)
                e.setMinimumWidth(72)
                grid_p.addWidget(e, row_i, col_i + 1)
                edits.append(e)
                # 베이스 포즈 편집 시 자동 재계산 트리거
                if pose_name in base_pose_keys:
                    e.editingFinished.connect(self._on_gap_param_changed)
            self._pose_edits[pose_name] = edits

        pose_count = len(_all_pose_names(num_tubes=3, num_trays=3))
        btn_p = QPushButton("포즈 적용")
        btn_p.setStyleSheet(apply_btn_style)
        btn_p.clicked.connect(self._on_apply_poses)
        grid_p.addWidget(btn_p, pose_count + 1, 0, 1, 7)
        layout.addWidget(grp_poses)

        self.btn_apply_robot_param.setText("전체 적용 (Doosan + Task + 포즈)")
        layout.addWidget(self.btn_apply_robot_param)
        layout.addStretch()

        # UI 빌드 직후 YAML 값 즉시 로드
        self._load_from_yaml()

    # -----------------------------------------------------------------
    # YAML 파일 직접 파싱 — 노드 연결 없이 즉시 초기값 채우기
    # -----------------------------------------------------------------
    def _load_from_yaml(self):
        import yaml
        try:
            config_path = os.path.join(
                get_package_share_directory('robot_control_pkg'),
                'config', 'robot_params.yaml'
            )
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.log(f"[경고] robot_params.yaml 로드 실패: {e}")
            return

        doosan = config.get('doosan_robot_control_node', {}).get('ros__parameters', {})
        task   = config.get('robot_task_manager_node',   {}).get('ros__parameters', {})

        # Doosan 스칼라
        for name, _ in _DOOSAN_SCALAR:
            if name in doosan:
                self._scalar_edits['doosan'][name].setText(str(doosan[name]))

        # Gap 계산 파라미터
        for name, default in _CALC_PARAMS:
            if name in doosan:
                self._calc_edits[name].setText(str(doosan[name]))

        # 포즈 (poses.*)
        for pose_name in _all_pose_names():
            key = f'poses.{pose_name}'
            if key in doosan:
                vals = doosan[key]
                for i, v in enumerate(vals[:6]):
                    self._pose_edits[pose_name][i].setText(str(v))

        # Task Manager 스칼라
        for name, _ in _TASK_SCALAR:
            if name in task:
                self._scalar_edits['task'][name].setText(str(task[name]))

        self.log("[로봇 파라미터] YAML 초기값 로드 완료")

    # -----------------------------------------------------------------
    # 초기값 로드 — 관리자 로그인 시 GetParameters 서비스로 실시간 갱신
    # -----------------------------------------------------------------
    def _load_robot_params(self):
        doosan_names = [name for name, _ in _DOOSAN_SCALAR]
        doosan_names += [f'poses.{n}' for n in _all_pose_names(num_tubes=3, num_trays=3)]

        if self.node.get_param_doosan_client.service_is_ready():
            fut = self.node.call_get_param_doosan(doosan_names)
            fut.add_done_callback(
                lambda f: self._on_doosan_params_loaded(f, doosan_names))
        else:
            self.log("[경고] doosan 파라미터 서비스 미준비 — 초기값 로드 생략")

        task_names = [name for name, _ in _TASK_SCALAR]
        if self.node.get_param_task_client.service_is_ready():
            fut = self.node.call_get_param_task(task_names)
            fut.add_done_callback(
                lambda f: self._on_task_params_loaded(f, task_names))
        else:
            self.log("[경고] task_manager 파라미터 서비스 미준비 — 초기값 로드 생략")

    def _on_doosan_params_loaded(self, future, names):
        # ROS 스핀 스레드에서 호출 — 위젯 직접 접근 금지, 시그널로 GUI 스레드에 전달
        try:
            result = future.result()
        except Exception as e:
            self.log_signal.emit(f"[GetParameters/doosan] 오류: {e}")
            return
        self._doosan_params_ready.emit(names, list(result.values))

    def _apply_doosan_params(self, names, pvalues):
        # GUI 스레드 슬롯 — 위젯 업데이트 안전
        scalar_keys = {name for name, _ in _DOOSAN_SCALAR}
        for name, pval in zip(names, pvalues):
            if name in scalar_keys:
                if pval.type == ParameterType.PARAMETER_DOUBLE:
                    self._scalar_edits['doosan'][name].setText(f"{pval.double_value:.4f}")
                elif pval.type == ParameterType.PARAMETER_INTEGER:
                    self._scalar_edits['doosan'][name].setText(str(pval.integer_value))
            elif name.startswith('poses.'):
                pose_key = name[6:]
                if pose_key in self._pose_edits and pval.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    for i, v in enumerate(list(pval.double_array_value)[:6]):
                        self._pose_edits[pose_key][i].setText(f"{v:.4f}")
        self.log("[로봇 파라미터] doosan 초기값 로드 완료")

    def _on_task_params_loaded(self, future, names):
        # ROS 스핀 스레드에서 호출 — 시그널로 GUI 스레드에 전달
        try:
            result = future.result()
        except Exception as e:
            self.log_signal.emit(f"[GetParameters/task] 오류: {e}")
            return
        self._task_params_ready.emit(names, list(result.values))

    def _apply_task_params(self, names, pvalues):
        # GUI 스레드 슬롯 — 위젯 업데이트 안전
        for name, pval in zip(names, pvalues):
            if name not in self._scalar_edits['task']:
                continue
            if pval.type == ParameterType.PARAMETER_DOUBLE:
                self._scalar_edits['task'][name].setText(f"{pval.double_value:.4f}")
            elif pval.type == ParameterType.PARAMETER_INTEGER:
                self._scalar_edits['task'][name].setText(str(pval.integer_value))
        self.log("[로봇 파라미터] task_manager 초기값 로드 완료")

    # -----------------------------------------------------------------
    # 자동 재계산 — tube_gap / tray_gap / *_crood 또는 베이스 포즈 변경 시
    #
    # 기준(base): tray_idx=0, tube_idx=0 포즈
    # refill_target_{t}_{u}_{type}:
    #   [tray_crood] = base[tray_crood] + t * tray_gap  (양방향)
    #   [tube_crood] = base[tube_crood] - u * tube_gap  (음방향)
    # dispose_tray_{t}_tube_{u}_{type}: 동일 공식
    # tray_transfer는 실측값이므로 자동 재계산 제외
    # -----------------------------------------------------------------
    def _on_gap_param_changed(self):
        self._recalc_derived_poses()

    def _recalc_derived_poses(self):
        try:
            tube_gap         = float(self._calc_edits['tube_gap'].text())
            tray_gap         = float(self._calc_edits['tray_gap'].text())
            tube_ax          = int(float(self._calc_edits['tube_crood'].text()))
            tray_ax          = int(float(self._calc_edits['tray_crood'].text()))
            work_z_offset    = float(self._calc_edits['work_z_offset'].text())
            pour_tube_offset = float(self._calc_edits['pour_tube_offset'].text())
            pour_z_offset    = float(self._calc_edits['pour_z_offset'].text())
            pour_ry_offset   = float(self._calc_edits['pour_ry_offset'].text())
            grip_z_offset    = float(self._calc_edits['grip_z_offset'].text())
        except ValueError:
            return

        if not (0 <= tube_ax <= 2 and 0 <= tray_ax <= 2):
            self.log("[경고] tube_crood/tray_crood 는 0(X)·1(Y)·2(Z) 중 하나여야 합니다.")
            return

        def read_pose(name):
            if name not in self._pose_edits:
                return None
            try:
                return [float(e.text()) for e in self._pose_edits[name]]
            except ValueError:
                return None

        def write_pose(name, vals):
            if name not in self._pose_edits:
                return
            for i, v in enumerate(vals):
                self._pose_edits[name][i].setText(f"{v:.4f}")

        refill_base  = read_pose('refill_target_0_0_approach_pose')
        dispose_base = read_pose('dispose_tray_0_tube_0_approach_pose')
        if refill_base is None or dispose_base is None:
            return

        for t in range(3):
            for u in range(3):
                # ① gap 적용 → 9개 approach 위치
                refill_ap = list(refill_base)
                refill_ap[tray_ax] += t * tray_gap
                refill_ap[tube_ax] -= u * tube_gap

                dispose_ap = list(dispose_base)
                dispose_ap[tray_ax] += t * tray_gap
                dispose_ap[tube_ax] -= u * tube_gap

                # ② work: approach에서 Z만 내림
                refill_work = list(refill_ap)
                refill_work[2] += work_z_offset

                # ③ pour: approach에서 튜브축 이동 + Z 내림 + RY 기울임
                refill_pour = list(refill_ap)
                refill_pour[tube_ax] += pour_tube_offset
                refill_pour[2]       += pour_z_offset
                refill_pour[4]       += pour_ry_offset  # RY

                # ④ grip: approach에서 Z만 내림
                dispose_grip = list(dispose_ap)
                dispose_grip[2] += grip_z_offset

                # approach 기준 포즈(0,0)는 사용자 직접 입력 — 덮어쓰지 않음
                if not (t == 0 and u == 0):
                    write_pose(f'refill_target_{t}_{u}_approach_pose', refill_ap)
                    write_pose(f'dispose_tray_{t}_tube_{u}_approach_pose', dispose_ap)

                # work/pour/grip은 기준(0,0) 포함 전체 재계산
                write_pose(f'refill_target_{t}_{u}_work_pose', refill_work)
                write_pose(f'refill_target_{t}_{u}_pour_pose', refill_pour)
                write_pose(f'dispose_tray_{t}_tube_{u}_grip_pose', dispose_grip)

    # -----------------------------------------------------------------
    # 파라미터 범위 검증 — 오류 메시지 리스트 반환 (빈 리스트 = OK)
    # -----------------------------------------------------------------
    def _validate_doosan_params(self):
        errors = []
        for name, (lo, hi) in _DOOSAN_SCALAR_RANGES.items():
            txt = self._scalar_edits['doosan'][name].text()
            try:
                val = float(txt)
                if not (lo <= val <= hi):
                    errors.append(f"{name}: {val}  (허용 {lo} ~ {hi})")
            except ValueError:
                errors.append(f"{name}: '{txt}' — 숫자가 아닙니다")
        return errors

    def _validate_task_params(self):
        errors = []
        for name, (lo, hi) in _TASK_SCALAR_RANGES.items():
            txt = self._scalar_edits['task'][name].text()
            try:
                val = float(txt)
                if not (lo <= val <= hi):
                    errors.append(f"{name}: {val}  (허용 {lo} ~ {hi})")
            except ValueError:
                errors.append(f"{name}: '{txt}' — 숫자가 아닙니다")
        return errors

    def _validate_params(self):
        return self._validate_doosan_params() + self._validate_task_params()

    # -----------------------------------------------------------------
    # 파라미터 빌드 헬퍼
    # -----------------------------------------------------------------
    def _make_param(self, name, ptype, text):
        pval = ParameterValue()
        if ptype == float:
            pval.type = ParameterType.PARAMETER_DOUBLE
            pval.double_value = float(text)
        else:
            pval.type = ParameterType.PARAMETER_INTEGER
            pval.integer_value = int(float(text))
        p = Parameter()
        p.name = name
        p.value = pval
        return p

    def _build_doosan_scalar_params(self):
        params = []
        for name, ptype in _DOOSAN_SCALAR:
            try:
                params.append(self._make_param(name, ptype, self._scalar_edits['doosan'][name].text()))
            except ValueError:
                self.log(f"[경고] {name} 파싱 실패 — 건너뜀")
        return params

    def _build_task_scalar_params(self):
        params = []
        for name, ptype in _TASK_SCALAR:
            try:
                params.append(self._make_param(name, ptype, self._scalar_edits['task'][name].text()))
            except ValueError:
                self.log(f"[경고] {name} 파싱 실패 — 건너뜀")
        return params

    def _build_pose_params(self):
        params = []
        for pose_name in _all_pose_names(num_tubes=3, num_trays=3):
            if pose_name not in self._pose_edits:
                continue
            try:
                vals = [float(e.text()) for e in self._pose_edits[pose_name]]
                pval = ParameterValue()
                pval.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                pval.double_array_value = vals
                p = Parameter()
                p.name = f'poses.{pose_name}'
                p.value = pval
                params.append(p)
            except ValueError:
                self.log(f"[경고] poses.{pose_name} 파싱 실패 — 건너뜀")
        return params

    # -----------------------------------------------------------------
    # Apply 완료 팝업 슬롯 (GUI 스레드에서 실행)
    # -----------------------------------------------------------------
    def _on_apply_complete(self, success, message):
        if success:
            QMessageBox.information(self, "파라미터 적용 완료", message)
        else:
            QMessageBox.warning(self, "파라미터 적용 실패",
                                f"일부 파라미터 적용에 실패했습니다:\n\n{message}")

    # -----------------------------------------------------------------
    # 2026-06-27 soo: Vision 기능 ON/OFF 토글 그룹박스
    # -----------------------------------------------------------------
    _BTN_ON  = ("background:#2E7D32; color:white; font-weight:bold;"
                " border-radius:4px; padding:8px 16px;")
    _BTN_OFF = ("background:#616161; color:white; font-weight:bold;"
                " border-radius:4px; padding:8px 16px;")

    def _update_feature_btn(self, btn: QPushButton, on: bool):
        btn.setChecked(on)
        btn.setText("ON" if on else "OFF")
        btn.setStyleSheet(self._BTN_ON if on else self._BTN_OFF)

    def _setup_vision_feature_toggles(self):
        grp = QGroupBox("Vision 기능")
        grid = QGridLayout()
        grid.setSpacing(10)
        grp.setLayout(grid)

        features = [
            ("카메라 스트리밍", self._on_toggle_camera),
            ("YOLO 추론",       self._on_toggle_yolo),
            ("높이 감지",       self._on_toggle_height),
            ("손 감지 안전",    self._on_toggle_hand),
        ]
        self.btn_feat_camera = None
        self.btn_feat_yolo   = None
        self.btn_feat_height = None
        self.btn_feat_hand   = None
        btn_refs = ['btn_feat_camera', 'btn_feat_yolo', 'btn_feat_height', 'btn_feat_hand']

        for i, (label, callback) in enumerate(features):
            lbl = QLabel(label)
            lbl.setStyleSheet("font-weight: bold;")
            btn = QPushButton("ON")
            btn.setCheckable(True)
            btn.setChecked(True)
            btn.setStyleSheet(self._BTN_ON)
            btn.clicked.connect(callback)
            setattr(self, btn_refs[i], btn)
            row, col = divmod(i, 2)
            grid.addWidget(lbl, row, col * 2)
            grid.addWidget(btn, row, col * 2 + 1)

        self.vl_motion_vision.insertWidget(0, grp)

    def _toggle_bool_param(self, client, call_fn, param_name, enabled, btn):
        def run():
            pval = ParameterValue()
            pval.type = ParameterType.PARAMETER_BOOL
            pval.bool_value = enabled
            p = Parameter()
            p.name = param_name
            p.value = pval
            if client.wait_for_service(timeout_sec=3.0):
                done_ev = threading.Event()
                fut = call_fn([p])
                def on_done(f):
                    try:
                        result = f.result()
                        failed = [r for r in result.results if not r.successful]
                        if failed:
                            self.log_signal.emit(f"[{param_name}] 설정 실패: {failed[0].reason}")
                    except Exception as e:
                        self.log_signal.emit(f"[{param_name}] 오류: {e}")
                    finally:
                        done_ev.set()
                fut.add_done_callback(on_done)
                done_ev.wait(timeout=5.0)
            else:
                self.log_signal.emit(f"[경고] 서비스 미응답 — {param_name} 설정 실패")
        threading.Thread(target=run, daemon=True).start()

    def _on_toggle_camera(self, checked):
        self._update_feature_btn(self.btn_feat_camera, checked)
        self.log(f"카메라 스트리밍: {'ON' if checked else 'OFF'}")
        self._toggle_bool_param(
            self.node.set_param_camera_client,
            self.node.call_set_param_camera,
            'streaming_enabled', checked, self.btn_feat_camera)

    def _on_toggle_yolo(self, checked):
        self._update_feature_btn(self.btn_feat_yolo, checked)
        self.node.publish_yolo_enabled(checked)
        self.log(f"YOLO 추론: {'ON' if checked else 'OFF'}")

    def _on_toggle_height(self, checked):
        self._update_feature_btn(self.btn_feat_height, checked)
        self.log(f"높이 감지: {'ON' if checked else 'OFF'}")
        self._toggle_bool_param(
            self.node.set_param_detector_client,
            self.node.call_set_param_detector,
            'height_publish_enabled', checked, self.btn_feat_height)

    def _on_toggle_hand(self, checked):
        self._update_feature_btn(self.btn_feat_hand, checked)
        self.log(f"손 감지 안전: {'ON' if checked else 'OFF'}")
        self._toggle_bool_param(
            self.node.set_param_detector_client,
            self.node.call_set_param_detector,
            'hand_safety_enabled', checked, self.btn_feat_hand)

    # -----------------------------------------------------------------
    # 비전 파라미터 UI 셋업 — 바운딩박스/높이 신뢰도 두 항목
    # -----------------------------------------------------------------
    def _setup_vision_conf_ui(self):
        self.lbl_vision_conf.setText("바운딩 박스 신뢰도")
        self.spin_vision_confidence.setRange(0.01, 1.0)
        self.spin_vision_confidence.setSingleStep(0.05)
        self.spin_vision_confidence.setDecimals(2)
        self.spin_vision_confidence.setValue(0.4)

        self.spin_vision_height_conf = QDoubleSpinBox()
        self.spin_vision_height_conf.setRange(0.01, 1.0)
        self.spin_vision_height_conf.setSingleStep(0.05)
        self.spin_vision_height_conf.setDecimals(2)
        self.spin_vision_height_conf.setValue(0.4)
        lbl_height = QLabel("높이 신뢰도")
        # spin_vision_confidence 바로 아래(row 1)에 삽입
        self.formLayout_vision.insertRow(1, lbl_height, self.spin_vision_height_conf)

        # 2026-06-27 soo: FPS 위젯 — 기존 spin_vision_fps 재활용, 라벨 변경
        self.lbl_vision_fps.setText("FPS")
        self.spin_vision_fps.setRange(5.0, 15.0)
        self.spin_vision_fps.setSingleStep(1.0)
        self.spin_vision_fps.setDecimals(0)
        self.spin_vision_fps.setValue(7.0)

        # 2026-06-27 soo: YOLO 모델 라디오 버튼 — weights 디렉토리 스캔
        self._model_btn_group = QButtonGroup(self)
        self._model_radio_map = {}  # 절대경로 → QRadioButton

        try:
            weights_dir = os.path.join(
                get_package_share_directory('vision_pkg'), 'weights')
            model_files = sorted(
                f for f in os.listdir(weights_dir)
                if f.endswith('.pt') or f.endswith('.onnx')
            )
        except Exception:
            weights_dir = ''
            model_files = []

        radio_widget = QWidget()
        radio_hl = QHBoxLayout(radio_widget)
        radio_hl.setContentsMargins(0, 0, 0, 0)
        radio_hl.setSpacing(16)

        for fname in model_files:
            fpath = os.path.join(weights_dir, fname)
            rb = QRadioButton(fname)
            self._model_btn_group.addButton(rb)
            self._model_radio_map[fpath] = rb
            radio_hl.addWidget(rb)

        if self._model_radio_map:
            list(self._model_radio_map.values())[0].setChecked(True)

        lbl_model = QLabel("YOLO 모델")
        lbl_model.setStyleSheet("font-weight: bold;")
        self.formLayout_vision.addRow(lbl_model, radio_widget)

        self.btn_apply_vision_admin.clicked.connect(self._on_apply_vision_conf)

    # -----------------------------------------------------------------
    # 비전 confidence 초기값 로드 — 관리자 로그인 시 호출
    # -----------------------------------------------------------------
    def _load_vision_params(self):
        # 2026-06-27 soo: side_camera_node FPS 로드
        if self.node.get_param_camera_client.service_is_ready():
            cam_names = ['publish_rate_hz', 'streaming_enabled']
            fut = self.node.call_get_param_camera(cam_names)
            fut.add_done_callback(
                lambda f: self._on_camera_params_loaded(f, cam_names))
        else:
            self.log("[경고] side_camera_node 파라미터 서비스 미준비 — FPS 초기값 로드 생략")

        if self.node.get_param_detector_client.service_is_ready():
            det_names = ['confidence_threshold', 'model_path', 'hand_safety_enabled', 'height_publish_enabled']
            fut = self.node.call_get_param_detector(det_names)
            fut.add_done_callback(
                lambda f: self._on_detector_params_loaded(f, det_names))
            # 2026-06-27 soo: ROI 파라미터 로드
            roi_names = ['roi_x_min_px', 'roi_x_max_px']
            fut_roi = self.node.call_get_param_detector(roi_names)
            fut_roi.add_done_callback(
                lambda f: self._on_detector_roi_params_loaded(f, roi_names))
            # 2026-06-27 soo: 버퍼 파라미터 별도 요청
            buf_names = [
                'height_buffer_size', 'height_publish_min_samples',
                'hand_detect_consecutive_frames', 'hand_lost_consecutive_frames',
            ]
            fut2 = self.node.call_get_param_detector(buf_names)
            fut2.add_done_callback(
                lambda f: self._on_detector_buffer_params_loaded(f, buf_names))
        else:
            self.log("[경고] liquid_height_detector 파라미터 서비스 미준비 — 초기값 로드 생략")

        if self.node.get_param_tube_state_client.service_is_ready():
            fut = self.node.call_get_param_tube_state(['confidence_threshold'])
            fut.add_done_callback(
                lambda f: self._on_tube_state_params_loaded(f, ['confidence_threshold']))
        else:
            self.log("[경고] tube_state_publisher 파라미터 서비스 미준비 — 초기값 로드 생략")

    def _on_detector_params_loaded(self, future, names):
        try:
            result = future.result()
        except Exception as e:
            self.log_signal.emit(f"[GetParameters/detector] 오류: {e}")
            return
        self._detector_params_ready.emit(names, list(result.values))

    def _apply_detector_params(self, names, pvalues):
        for name, pval in zip(names, pvalues):
            if name == 'confidence_threshold' and pval.type == ParameterType.PARAMETER_DOUBLE:
                self.spin_vision_confidence.setValue(pval.double_value)
            elif name == 'model_path' and pval.type == ParameterType.PARAMETER_STRING:
                rb = self._model_radio_map.get(pval.string_value)
                if rb:
                    rb.setChecked(True)
            elif name == 'hand_safety_enabled' and pval.type == ParameterType.PARAMETER_BOOL:  # 2026-06-27 soo
                self._update_feature_btn(self.btn_feat_hand, pval.bool_value)
            elif name == 'height_publish_enabled' and pval.type == ParameterType.PARAMETER_BOOL:  # 2026-06-27 soo
                self._update_feature_btn(self.btn_feat_height, pval.bool_value)
        self.log("[비전 파라미터] confidence / 모델 / 기능 상태 로드 완료")

    def _on_tube_state_params_loaded(self, future, names):
        try:
            result = future.result()
        except Exception as e:
            self.log_signal.emit(f"[GetParameters/tube_state] 오류: {e}")
            return
        self._tube_state_params_ready.emit(names, list(result.values))

    def _apply_tube_state_params(self, names, pvalues):
        for name, pval in zip(names, pvalues):
            if name == 'confidence_threshold' and pval.type == ParameterType.PARAMETER_DOUBLE:
                self.spin_vision_height_conf.setValue(pval.double_value)
        self.log("[비전 파라미터] 높이 confidence 로드 완료")

    def _on_camera_params_loaded(self, future, names):
        # 2026-06-27 soo
        try:
            result = future.result()
        except Exception as e:
            self.log_signal.emit(f"[GetParameters/camera] 오류: {e}")
            return
        self._camera_params_ready.emit(names, list(result.values))

    def _apply_camera_params(self, names, pvalues):
        # 2026-06-27 soo
        for name, pval in zip(names, pvalues):
            if name == 'publish_rate_hz' and pval.type == ParameterType.PARAMETER_DOUBLE:
                self.spin_vision_fps.setValue(pval.double_value)
            elif name == 'streaming_enabled' and pval.type == ParameterType.PARAMETER_BOOL:
                self._update_feature_btn(self.btn_feat_camera, pval.bool_value)
        self.log("[비전 파라미터] FPS / 카메라 상태 로드 완료")

    def _on_detector_roi_params_loaded(self, future, names):
        # 2026-06-27 soo
        try:
            result = future.result()
        except Exception as e:
            self.log_signal.emit(f"[GetParameters/detector_roi] 오류: {e}")
            return
        self._detector_roi_params_ready.emit(names, list(result.values))

    def _apply_detector_roi_params(self, names, pvalues):
        # 2026-06-27 soo: GUI 스레드 슬롯
        for name, pval in zip(names, pvalues):
            if pval.type == ParameterType.PARAMETER_DOUBLE:
                val = int(pval.double_value)
                if name == 'roi_x_min_px':
                    self.spin_roi_x_min.setValue(val)
                elif name == 'roi_x_max_px':
                    self.spin_roi_x_max.setValue(val)
        self.log("[비전 파라미터] 감지 영역 X 초기값 로드 완료")

    # -----------------------------------------------------------------
    # 2026-06-27 soo: 추론 안정화 버퍼 UI 셋업 — confidence 그룹박스 아래에 동적 생성
    # -----------------------------------------------------------------
    def _setup_vision_buffer_ui(self):
        grp = QGroupBox("추론 안정화 버퍼")
        form = QFormLayout()
        grp.setLayout(form)

        self.spin_buf_size = QSpinBox()
        self.spin_buf_size.setRange(1, 30)
        self.spin_buf_size.setValue(5)
        form.addRow("추론 버퍼 크기 (프레임)", self.spin_buf_size)

        self.spin_buf_min_samples = QSpinBox()
        self.spin_buf_min_samples.setRange(1, 30)
        self.spin_buf_min_samples.setValue(3)
        form.addRow("최소 유효 샘플 수", self.spin_buf_min_samples)

        self.spin_hand_detect_frames = QSpinBox()
        self.spin_hand_detect_frames.setRange(1, 20)
        self.spin_hand_detect_frames.setValue(3)
        form.addRow("손 감지 연속 프레임", self.spin_hand_detect_frames)

        self.spin_hand_lost_frames = QSpinBox()
        self.spin_hand_lost_frames.setRange(1, 20)
        self.spin_hand_lost_frames.setValue(2)
        form.addRow("손 해제 연속 프레임", self.spin_hand_lost_frames)

        btn_apply = QPushButton("버퍼 적용")
        btn_apply.clicked.connect(self._on_apply_vision_buffer)
        form.addRow(btn_apply)

        # spacer 바로 앞(마지막 - 1)에 삽입
        count = self.vl_motion_vision.count()
        self.vl_motion_vision.insertWidget(count - 1, grp)

    def _on_detector_buffer_params_loaded(self, future, names):
        try:
            result = future.result()
        except Exception as e:
            self.log_signal.emit(f"[GetParameters/detector_buffer] 오류: {e}")
            return
        self._detector_buffer_params_ready.emit(names, list(result.values))

    def _apply_detector_buffer_params(self, names, pvalues):
        mapping = {
            'height_buffer_size':           self.spin_buf_size,
            'height_publish_min_samples':   self.spin_buf_min_samples,
            'hand_detect_consecutive_frames': self.spin_hand_detect_frames,
            'hand_lost_consecutive_frames': self.spin_hand_lost_frames,
        }
        for name, pval in zip(names, pvalues):
            if name in mapping and pval.type == ParameterType.PARAMETER_INTEGER:
                mapping[name].setValue(pval.integer_value)
        self.log("[비전 파라미터] 추론 버퍼 초기값 로드 완료")

    def _on_apply_vision_buffer(self):
        params = {
            'height_buffer_size':             self.spin_buf_size.value(),
            'height_publish_min_samples':     self.spin_buf_min_samples.value(),
            'hand_detect_consecutive_frames': self.spin_hand_detect_frames.value(),
            'hand_lost_consecutive_frames':   self.spin_hand_lost_frames.value(),
        }
        self.log("추론 버퍼 파라미터 적용 요청...")
        threading.Thread(
            target=self._apply_vision_buffer_thread,
            args=(params,),
            daemon=True,
        ).start()

    def _apply_vision_buffer_thread(self, params):
        all_errors = []
        total_applied = 0

        def make_int_param(name, value):
            pval = ParameterValue()
            pval.type = ParameterType.PARAMETER_INTEGER
            pval.integer_value = int(value)
            p = Parameter()
            p.name = name
            p.value = pval
            return p

        def call_and_wait(call_fn, ros_params, node_name):
            nonlocal total_applied
            done_ev = threading.Event()
            fut = call_fn(ros_params)

            def on_done(f):
                nonlocal total_applied
                try:
                    result = f.result()
                    failed = [r for r in result.results if not r.successful]
                    if failed:
                        for r in failed:
                            all_errors.append(f"[{node_name}] {r.reason}")
                            self.log_signal.emit(f"[{node_name}] 설정 실패: {r.reason}")
                    else:
                        total_applied += len(result.results)
                        self.log_signal.emit(
                            f"[{node_name}] 버퍼 파라미터 {len(result.results)}개 적용 완료")
                except Exception as e:
                    all_errors.append(f"[{node_name}] 오류: {e}")
                    self.log_signal.emit(f"[{node_name}] SetParameters 오류: {e}")
                finally:
                    done_ev.set()

            fut.add_done_callback(on_done)
            done_ev.wait(timeout=5.0)

        ros_params = [make_int_param(k, v) for k, v in params.items()]
        if self.node.set_param_detector_client.wait_for_service(timeout_sec=3.0):
            call_and_wait(
                self.node.call_set_param_detector, ros_params, 'liquid_height_detector')
        else:
            all_errors.append("[liquid_height_detector] 서비스 미응답")
            self.log_signal.emit("[경고] liquid_height_detector SetParameters 서비스 응답 없음")

        if all_errors:
            self._apply_complete_signal.emit(False, "\n".join(all_errors))
        else:
            self._apply_complete_signal.emit(True, f"추론 버퍼 {total_applied}개 적용 완료")

    # -----------------------------------------------------------------
    # 비전 confidence 적용 버튼 콜백
    # -----------------------------------------------------------------
    def _on_apply_vision_conf(self):
        det_conf   = self.spin_vision_confidence.value()
        ts_conf    = self.spin_vision_height_conf.value()
        fps        = self.spin_vision_fps.value()
        roi_x_min  = float(self.spin_roi_x_min.value())
        roi_x_max  = float(self.spin_roi_x_max.value())
        # 2026-06-27 soo: 선택된 라디오 버튼에서 모델 경로 추출
        selected_model = next(
            (path for path, rb in self._model_radio_map.items() if rb.isChecked()), None)
        self.log(
            f"비전 파라미터 적용 요청 — FPS: {fps:.0f}, "
            f"바운딩박스: {det_conf:.2f}, 높이: {ts_conf:.2f}, "
            f"감지영역 X: {roi_x_min:.0f}~{roi_x_max:.0f}, "
            f"모델: {os.path.basename(selected_model) if selected_model else '없음'}"
        )
        threading.Thread(
            target=self._apply_vision_conf_thread,
            args=(det_conf, ts_conf, fps, roi_x_min, roi_x_max, selected_model),
            daemon=True,
        ).start()

    def _apply_vision_conf_thread(self, det_conf, ts_conf, fps, roi_x_min, roi_x_max, model_path):
        # 2026-06-27 soo: fps, roi, model_path 인자 추가
        all_errors = []
        total_applied = 0

        def make_double_param(name, value):
            pval = ParameterValue()
            pval.type = ParameterType.PARAMETER_DOUBLE
            pval.double_value = value
            p = Parameter()
            p.name = name
            p.value = pval
            return p

        def call_and_wait(call_fn, params, node_name):
            nonlocal total_applied
            done_ev = threading.Event()
            fut = call_fn(params)

            def on_done(f):
                nonlocal total_applied
                try:
                    result = f.result()
                    failed = [r for r in result.results if not r.successful]
                    if failed:
                        for r in failed:
                            all_errors.append(f"[{node_name}] {r.reason}")
                            self.log_signal.emit(f"[{node_name}] 설정 실패: {r.reason}")
                    else:
                        total_applied += len(result.results)
                        self.log_signal.emit(f"[{node_name}] 적용 완료")
                except Exception as e:
                    all_errors.append(f"[{node_name}] 오류: {e}")
                    self.log_signal.emit(f"[{node_name}] SetParameters 오류: {e}")
                finally:
                    done_ev.set()

            fut.add_done_callback(on_done)
            done_ev.wait(timeout=5.0)

        # FPS — side_camera_node
        if self.node.set_param_camera_client.wait_for_service(timeout_sec=3.0):
            call_and_wait(
                self.node.call_set_param_camera,
                [make_double_param('publish_rate_hz', fps)],
                'side_camera_node')
        else:
            all_errors.append("[side_camera_node] 서비스 미응답")
            self.log_signal.emit("[경고] side_camera_node SetParameters 서비스 응답 없음")

        if self.node.set_param_detector_client.wait_for_service(timeout_sec=3.0):
            # 2026-06-27 soo: model_path는 STRING 타입으로 별도 빌드
            det_params = [
                make_double_param('confidence_threshold', det_conf),
                make_double_param('roi_x_min_px', roi_x_min),
                make_double_param('roi_x_max_px', roi_x_max),
            ]
            if model_path:
                mp = Parameter()
                mp.name = 'model_path'
                mp.value = ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value=model_path)
                det_params.append(mp)
            call_and_wait(
                self.node.call_set_param_detector,
                det_params,
                'liquid_height_detector')
        else:
            all_errors.append("[liquid_height_detector] 서비스 미응답")
            self.log_signal.emit("[경고] liquid_height_detector SetParameters 서비스 응답 없음")

        if self.node.set_param_tube_state_client.wait_for_service(timeout_sec=3.0):
            call_and_wait(
                self.node.call_set_param_tube_state,
                [make_double_param('confidence_threshold', ts_conf)],
                'tube_state_publisher')
        else:
            all_errors.append("[tube_state_publisher] 서비스 미응답")
            self.log_signal.emit("[경고] tube_state_publisher SetParameters 서비스 응답 없음")

        if all_errors:
            self._apply_complete_signal.emit(False, "\n".join(all_errors))
        else:
            self._apply_complete_signal.emit(True, f"비전 파라미터 {total_applied}개 적용 완료")

    # -----------------------------------------------------------------
    # 그룹별 Apply 버튼 콜백
    # -----------------------------------------------------------------
    def _on_apply_doosan_scalar(self):
        errors = self._validate_doosan_params()
        if errors:
            QMessageBox.warning(self, "파라미터 검증 실패",
                                "다음 항목의 값을 확인해주세요:\n\n" + "\n".join(f"• {e}" for e in errors))
            return
        params = self._build_doosan_scalar_params()
        self.log(f"Doosan 파라미터 적용 요청 ({len(params)}개)...")
        threading.Thread(target=self._apply_params_thread, args=(params, []), daemon=True).start()

    def _on_apply_task_scalar(self):
        errors = self._validate_task_params()
        if errors:
            QMessageBox.warning(self, "파라미터 검증 실패",
                                "다음 항목의 값을 확인해주세요:\n\n" + "\n".join(f"• {e}" for e in errors))
            return
        params = self._build_task_scalar_params()
        self.log(f"Task 파라미터 적용 요청 ({len(params)}개)...")
        threading.Thread(target=self._apply_params_thread, args=([], params), daemon=True).start()

    def _on_apply_poses(self):
        params = self._build_pose_params()
        self.log(f"포즈 적용 요청 ({len(params)}개)...")
        threading.Thread(target=self._apply_params_thread, args=(params, []), daemon=True).start()

    # -----------------------------------------------------------------
    # 전체 Apply 버튼 콜백 (Doosan 스칼라 + 포즈 + Task 한번에)
    # -----------------------------------------------------------------
    def on_apply_robot_param(self):
        errors = self._validate_params()
        if errors:
            QMessageBox.warning(
                self, "파라미터 검증 실패",
                "다음 항목의 값을 확인해주세요:\n\n" + "\n".join(f"• {e}" for e in errors)
            )
            return

        doosan_params = self._build_doosan_scalar_params() + self._build_pose_params()
        task_params   = self._build_task_scalar_params()

        self.log(f"전체 파라미터 적용 요청 (doosan {len(doosan_params)}개 / task {len(task_params)}개) ...")
        threading.Thread(
            target=self._apply_params_thread,
            args=(doosan_params, task_params),
            daemon=True,
        ).start()

    def _apply_params_thread(self, doosan_params, task_params):
        all_errors = []
        total_applied = 0

        def call_and_wait(client, call_fn, params, node_name):
            nonlocal total_applied
            done_ev = threading.Event()
            fut = call_fn(params)

            def on_done(f):
                nonlocal total_applied
                try:
                    result = f.result()
                    failed = [r for r in result.results if not r.successful]
                    if failed:
                        for r in failed:
                            all_errors.append(f"[{node_name}] {r.reason}")
                            self.log_signal.emit(f"[{node_name}] 설정 실패: {r.reason}")
                    else:
                        total_applied += len(result.results)
                        self.log_signal.emit(
                            f"[{node_name}] 파라미터 {len(result.results)}개 전체 적용 완료")
                except Exception as e:
                    all_errors.append(f"[{node_name}] 오류: {e}")
                    self.log_signal.emit(f"[{node_name}] SetParameters 오류: {e}")
                finally:
                    done_ev.set()

            fut.add_done_callback(on_done)
            done_ev.wait(timeout=10.0)

        if doosan_params:
            if self.node.set_param_doosan_client.wait_for_service(timeout_sec=3.0):
                call_and_wait(
                    self.node.set_param_doosan_client,
                    self.node.call_set_param_doosan,
                    doosan_params, 'doosan')
            else:
                all_errors.append("[doosan] 서비스 미응답 — doosan_robot_control_node 실행 확인")
                self.log_signal.emit(
                    "[경고] doosan SetParameters 서비스 응답 없음 — "
                    "doosan_robot_control_node 실행 중인지 확인하세요")

        if task_params:
            if self.node.set_param_task_client.wait_for_service(timeout_sec=3.0):
                call_and_wait(
                    self.node.set_param_task_client,
                    self.node.call_set_param_task,
                    task_params, 'task_manager')
            else:
                all_errors.append("[task_manager] 서비스 미응답 — robot_task_manager_node 실행 확인")
                self.log_signal.emit(
                    "[경고] task_manager SetParameters 서비스 응답 없음 — "
                    "robot_task_manager_node 실행 중인지 확인하세요")

        if all_errors:
            self._apply_complete_signal.emit(False, "\n".join(all_errors))
        else:
            self._apply_complete_signal.emit(
                True, f"파라미터 {total_applied}개 전체 적용 완료")


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedHMINode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = HMIDashboardApp(node)
    window.show()

    app_exec_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(app_exec_code)


if __name__ == "__main__":
    main()
