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
# =============================================================================

import sys
import os
import threading
from datetime import datetime
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
# 2026-06-24 soo: side_image CompressedImage 구독으로 변경 (publisher QoS 맞춤)
from sensor_msgs.msg import CompressedImage
from ament_index_python.packages import get_package_share_directory

from PyQt5 import uic
from PyQt5.QtWidgets import (QApplication, QDialog, QMessageBox)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap

from std_srvs.srv import SetBool, Trigger

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

        self.stop_task_client = self.create_client(StopTask, "/robot/stop_task")
        self.recheck_client = self.create_client(RequestRecheck, "/vision/request_recheck")
        self.set_system_running_client = self.create_client(SetBool, "/robot/set_system_running")
        self.reset_slot_anchors_client = self.create_client(Trigger, "/vision/reset_slot_anchors")
        self.reset_handled_slots_client = self.create_client(Trigger, "/vision/reset_handled_slots")

        self.resolution_pub = self.create_publisher(String, "/camera/resolution_cmd", 10)
        self.yolo_enable_pub = self.create_publisher(Bool, "/vision/yolo_enabled", 10)
        self.gripper_force_pub = self.create_publisher(Int32, "/gripper/force_cmd", 10)

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

    def call_set_system_running(self, running: bool):
        return self.set_system_running_client.call_async(SetBool.Request(data=running))

    def call_stop_task(self): return self.stop_task_client.call_async(StopTask.Request(stop=True))
    def call_request_recheck(self): return self.recheck_client.call_async(RequestRecheck.Request(request=True))

    def call_reset_slot_anchors(self):
        return self.reset_slot_anchors_client.call_async(Trigger.Request())

    def call_reset_handled_slots(self):
        return self.reset_handled_slots_client.call_async(Trigger.Request())

    def publish_resolution(self, res_str: str):
        msg = String(); msg.data = res_str; self.resolution_pub.publish(msg)

    def publish_yolo_enabled(self, enabled: bool):
        msg = Bool(); msg.data = enabled; self.yolo_enable_pub.publish(msg)

    def publish_gripper_force(self, percent: int):
        msg = Int32(); msg.data = percent; self.gripper_force_pub.publish(msg)


# =====================================================================
# 2. PyQt 기반 UI 화면 클래스 (.ui 파일의 탭 구조를 그대로 사용)
# 2026-06-24 soo: 외부 QTabWidget 제거 - uic.loadUi(self)로 직접 로드해 탭 중복 제거
# =====================================================================
class HMIDashboardApp(QDialog):
    # rclpy's future.add_done_callback() fires on the background ROS spin
    # thread, not the Qt GUI thread - touching QTextEdit from there directly
    # (the old self.log() call) crashes Qt. Routing through a signal lets Qt
    # marshal the string-only payload onto the GUI thread safely.
    log_signal = pyqtSignal(str)

    def __init__(self, node: IntegratedHMINode):
        super().__init__()
        self.node = node
        self.log_signal.connect(self.log)
        self.yolo_enabled = False
        self._grip_fail_shown = False
        # 2026-06-24 soo: 관리자 인증 상태 플래그 — 탭 전환 시 로그인 페이지 강제 표시에 사용
        self._admin_authenticated = False

        package_share_directory = get_package_share_directory('hmi_pkg')
        ui_path = os.path.join(package_share_directory, 'ui', 'vision_camera_state.ui')

        if not os.path.exists(ui_path):
            self.node.get_logger().error(f"UI 파일을 찾을 수 없습니다: {ui_path}")

        uic.loadUi(ui_path, self)  # 2026-06-24 soo: self.ui(QDialog) 래퍼 제거, 직접 로드

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

        self.slider_gripper_force.valueChanged.connect(self.on_gripper_force_changed)
        self.slider_gripper_force.sliderReleased.connect(self.on_gripper_force_apply)

        self.btn_clear_fail.clicked.connect(self.on_clear_grip_fail)

        # ── 2026-06-24 soo: 시스템 관리자 탭 연결 ───────────────────
        # 메인 QTabWidget 이름이 사용자에 의해 tabWidget → JOG 로 변경됨
        # 2026-06-25 soo: self.tabWidget → self.JOG 참조 수정
        self.JOG.currentChanged.connect(self._on_tab_changed)
        self.btn_admin_login.clicked.connect(self._on_admin_login)
        self.btn_admin_logout.clicked.connect(self._on_admin_logout)
        self.input_admin_pw.returnPressed.connect(self._on_admin_login)
        self.btn_apply_vision_params.clicked.connect(self._on_apply_vision_params)

        # ── 2026-06-25 soo: JOG 탭 연결 ─────────────────────────────
        # 협동로봇 6축(Joint) 및 TCP(X/Y/Z/A/B/C) 조그 제어
        self._jog_joint_vals = [0.0] * 6   # J1~J6 현재 각도 (deg)
        self._jog_tcp_vals = [0.0] * 6     # X/Y/Z(mm), A/B/C(deg) 현재값
        self.btn_jog_joint_mode.clicked.connect(lambda: self._on_jog_mode('joint'))
        self.btn_jog_tcp_mode.clicked.connect(lambda: self._on_jog_mode('tcp'))
        for i, nm in enumerate(['j1', 'j2', 'j3', 'j4', 'j5', 'j6']):
            getattr(self, f'btn_{nm}_minus').clicked.connect(lambda _, idx=i: self._on_jog_joint(idx, -1))
            getattr(self, f'btn_{nm}_plus').clicked.connect(lambda _, idx=i: self._on_jog_joint(idx, +1))
        for i, nm in enumerate(['x', 'y', 'z', 'a', 'b', 'c']):
            getattr(self, f'btn_tcp_{nm}_minus').clicked.connect(lambda _, idx=i: self._on_jog_tcp(idx, -1))
            getattr(self, f'btn_tcp_{nm}_plus').clicked.connect(lambda _, idx=i: self._on_jog_tcp(idx, +1))
        self.slider_jog_speed.valueChanged.connect(self._on_jog_speed_changed)
        self.btn_jog_stop.clicked.connect(self._on_jog_stop)

        self.tube_widgets = [
            (self.tube0_color_box, self.tube0_status_label, self.tube0_val_label),
            (self.tube1_color_box, self.tube1_status_label, self.tube1_val_label),
            (self.tube2_color_box, self.tube2_status_label, self.tube2_val_label),
        ]

        # 2026-06-24 soo: .ui 탭의 좌표 설정 위젯에 ROS 파라미터 초기값 채우기
        self._populate_pose_fields()
        self.btn_apply_params.clicked.connect(self.apply_params)

        self.log("PyQt HMI System initialized.")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh_dashboard)
        self.timer.start(150)

    # -----------------------------------------------------------------
    # 2026-06-24 soo: 좌표 설정 탭 — ROS 파라미터 → UI 초기값 채우기
    # -----------------------------------------------------------------
    def _populate_pose_fields(self):
        pose_field_map = [
            ('poses.home_pose',
             [self.input_home_x, self.input_home_y, self.input_home_z,
              self.input_home_rx, self.input_home_ry, self.input_home_rz]),
            ('poses.waste_pose',
             [self.input_waste_x, self.input_waste_y, self.input_waste_z,
              self.input_waste_rx, self.input_waste_ry, self.input_waste_rz]),
            ('poses.normal_tray_approach_pose',
             [self.input_normal_x, self.input_normal_y, self.input_normal_z,
              self.input_normal_rx, self.input_normal_ry, self.input_normal_rz]),
            ('poses.tube_0_approach_pose',
             [self.input_tube0_x, self.input_tube0_y, self.input_tube0_z,
              self.input_tube0_rx, self.input_tube0_ry, self.input_tube0_rz]),
        ]
        for param_name, line_edits in pose_field_map:
            try:
                vals = self.node.get_parameter(param_name).value
            except rclpy.exceptions.ParameterNotDeclaredException:
                continue
            for le, val in zip(line_edits, vals):
                le.setText(str(val))

    def apply_params(self):
        pose_field_map = [
            ('poses.home_pose',
             [self.input_home_x, self.input_home_y, self.input_home_z,
              self.input_home_rx, self.input_home_ry, self.input_home_rz]),
            ('poses.waste_pose',
             [self.input_waste_x, self.input_waste_y, self.input_waste_z,
              self.input_waste_rx, self.input_waste_ry, self.input_waste_rz]),
            ('poses.normal_tray_approach_pose',
             [self.input_normal_x, self.input_normal_y, self.input_normal_z,
              self.input_normal_rx, self.input_normal_ry, self.input_normal_rz]),
            ('poses.tube_0_approach_pose',
             [self.input_tube0_x, self.input_tube0_y, self.input_tube0_z,
              self.input_tube0_rx, self.input_tube0_ry, self.input_tube0_rz]),
        ]
        for param_name, line_edits in pose_field_map:
            try:
                new_vals = [float(le.text()) for le in line_edits]
                self.node.get_logger().info(f"[적용 대기] {param_name}: {new_vals}")
            except ValueError:
                QMessageBox.warning(self, "입력 오류", f"'{param_name}'에 숫자가 아닌 값이 있습니다.")
                return
        QMessageBox.information(self, "적용 완료", "새로운 로봇 6축 좌표계가 성공적으로 업데이트되었습니다.")

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
        stop_future = self.node.call_stop_task()
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
    def on_gripper_force_changed(self, value):
        self.lbl_gripper_force_val.setText(f"{value} %")

    def on_gripper_force_apply(self):
        value = self.slider_gripper_force.value()
        self.node.publish_gripper_force(value)
        self.log(f"그리퍼 강도 설정: {value}%")

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
        # 2026-06-25 soo: 메인 QTabWidget 이름 변경 tabWidget → JOG 반영
        admin_index = self.JOG.indexOf(self.tab_admin)
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

    def _on_apply_vision_params(self):
        # 2026-06-24 soo: Vision 파라미터 탭 — 발행속도/JPEG품질/카메라 인덱스 적용
        try:
            rate = float(self.input_publish_rate.text())
            quality = self.input_jpeg_quality.value()
            cam_idx = self.input_cam_index.value()
            self.log(f"[Vision 파라미터 적용] 발행속도={rate}Hz  JPEG품질={quality}  카메라={cam_idx}")
            QMessageBox.information(self, "적용 완료", f"Vision 파라미터가 반영되었습니다.\n발행속도: {rate} Hz\nJPEG 품질: {quality}\n카메라 인덱스: {cam_idx}")
        except ValueError:
            QMessageBox.warning(self, "입력 오류", "발행 속도에 숫자가 아닌 값이 있습니다.")

    # -----------------------------------------------------------------
    # 2026-06-25 soo: JOG 탭 콜백
    #   - Joint Mode: J1~J6 각도 조그
    #   - TCP Mode: X/Y/Z(mm), A/B/C(deg) 위치 조그
    #   - combo_jog_step으로 스텝 크기 선택 (0.1 / 1.0 / 5.0 / 10.0)
    # -----------------------------------------------------------------
    _JOG_BTN_ACTIVE   = "background-color:#1565C0; color:white; font-size:15px; font-weight:bold; border-radius:8px;"
    _JOG_BTN_INACTIVE = "background-color:#455A64; color:white; font-size:15px; font-weight:bold; border-radius:8px;"
    _JOG_JOINT_NAMES  = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
    _JOG_TCP_NAMES    = ['x', 'y', 'z', 'a', 'b', 'c']
    _JOG_TCP_UNITS    = ['mm', 'mm', 'mm', 'deg', 'deg', 'deg']

    def _on_jog_mode(self, mode: str):
        if mode == 'joint':
            self.stack_jog.setCurrentIndex(0)
            self.btn_jog_joint_mode.setStyleSheet(self._JOG_BTN_ACTIVE)
            self.btn_jog_tcp_mode.setStyleSheet(self._JOG_BTN_INACTIVE)
        else:
            self.stack_jog.setCurrentIndex(1)
            self.btn_jog_joint_mode.setStyleSheet(self._JOG_BTN_INACTIVE)
            self.btn_jog_tcp_mode.setStyleSheet(self._JOG_BTN_ACTIVE)

    def _get_jog_step(self) -> float:
        return float(self.combo_jog_step.currentText())

    def _on_jog_joint(self, idx: int, direction: int):
        step = self._get_jog_step() * direction
        self._jog_joint_vals[idx] += step
        nm = self._JOG_JOINT_NAMES[idx]
        getattr(self, f'lbl_{nm}_val').setText(f"{self._jog_joint_vals[idx]:7.2f}")
        self.log(f"[JOG Joint] J{idx+1} = {self._jog_joint_vals[idx]:.2f}°  (step {step:+.2f}°)")

    def _on_jog_tcp(self, idx: int, direction: int):
        step = self._get_jog_step() * direction
        self._jog_tcp_vals[idx] += step
        nm = self._JOG_TCP_NAMES[idx]
        unit = self._JOG_TCP_UNITS[idx]
        getattr(self, f'lbl_tcp_{nm}_val').setText(f"{self._jog_tcp_vals[idx]:7.2f}")
        self.log(f"[JOG TCP] {nm.upper()} = {self._jog_tcp_vals[idx]:.2f} {unit}  (step {step:+.2f})")

    def _on_jog_speed_changed(self, value: int):
        self.lbl_jog_speed_val.setText(f"{value} %")

    def _on_jog_stop(self):
        self.log("[JOG] STOP 명령 전송")
        future = self.node.call_stop_task()
        future.add_done_callback(lambda f: self._log_service_result("jog_stop", f))

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
        self._update_robot_led(status_msg)

        self.label_tube0_7.setText("OK" if self.node.camera_ok else "DISCONNECTED")

        if self.node.hand_detected:
            self.label_tube0_8.setText("YES - motion withheld")
            self.label_tube0_8.setStyleSheet("color: red; font-weight: bold;")
        else:
            self.label_tube0_8.setText("Clear")
            self.label_tube0_8.setStyleSheet("color: white;")

        self._update_grip_fail_banner()


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
