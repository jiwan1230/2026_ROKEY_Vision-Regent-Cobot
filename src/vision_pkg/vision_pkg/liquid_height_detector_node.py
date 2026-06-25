"""YOLOv8n-based liquid height inference node.

Subscribes to the side-view camera image, runs the fine-tuned YOLOv8n model
(classes: cup, height, hand) and publishes, for each of the num_tubes tube
slots, the calibrated liquid height and detection confidence. Also acts as
a safety sensor: publishes /vision/hand_detected when a hand is seen in the
work area, and /vision/camera_status if frames stop arriving (per the
PDF's Vision error-handling table in section 16).
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from interfaces.msg import TubeHeight
# 260624 jiwan bbox_utils.py 수정에 따른 import 문 수정 + 신뢰도를 위한 buffer를 위한 deque 추가
# from vision_pkg.bbox_utils import Box, assign_tube_zones, liquid_fill_fraction, match_height_box
from collections import deque
import statistics
from vision_pkg.bbox_utils import Box, match_cups_to_anchors, liquid_fill_fraction, match_height_box

# QoS 맞추기 위한 설정
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# end

# 260624 jiwan slot anchor 수동 리셋용 (새 트레이로 전환 시 호출, 지금은 수동 훅)
from std_srvs.srv import Trigger
# end

# 260624 jiwan side_image가 Image -> CompressedImage(JPEG)로 바뀜에 따른 import 수정
# from sensor_msgs.msg import Image
# from vision_pkg.ros_image_utils import image_to_bgr8
from sensor_msgs.msg import CompressedImage, Image
from vision_pkg.ros_image_utils import compressed_image_to_bgr8, bgr8_to_image
# end


class LiquidHeightDetectorNode(Node):
    def __init__(self):
        super().__init__("liquid_height_detector_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("confidence_threshold", 0.4)
        self.declare_parameter("num_tubes", 3)

        # 260624 jiwan 더이상 사용하지 않는 파라미터
        # self.declare_parameter("tube_height_mm", 100.0)
        # end

        self.declare_parameter("hand_safety_enabled", True)
        self.declare_parameter("camera_timeout_sec", 3.0)

        # 260624 파라미터 추가
        self.declare_parameter("yolo_imgsz", 640)
        self.declare_parameter("yolo_iou_threshold", 0.5)
        self.declare_parameter("yolo_max_det", 20)

        self.declare_parameter("height_buffer_size", 5)
        self.declare_parameter("height_publish_min_samples", 3)
        self.declare_parameter("height_filter_method", "median")

        self.declare_parameter("hand_detect_consecutive_frames", 3)
        self.declare_parameter("hand_lost_consecutive_frames", 2)

        # 260624 jiwan 동적 slot anchor 매칭용 tolerance (cup이 처음 num_tubes개
        # 동시에 보였을 때 부트스트랩한 anchor와 비교하는 허용 오차, px 단위)
        self.declare_parameter("slot_x_tolerance_px", 80.0)
        self.declare_parameter("row_y_tolerance_px", 35.0)
        # end

        self.yolo_imgsz = int(self.get_parameter("yolo_imgsz").value)
        self.yolo_iou_threshold = float(self.get_parameter("yolo_iou_threshold").value)
        self.yolo_max_det = int(self.get_parameter("yolo_max_det").value)

        self.height_buffer_size = int(self.get_parameter("height_buffer_size").value)
        self.height_publish_min_samples = int(self.get_parameter("height_publish_min_samples").value)
        self.height_filter_method = self.get_parameter("height_filter_method").value

        self.slot_x_tolerance_px = float(self.get_parameter("slot_x_tolerance_px").value)
        self.row_y_tolerance_px = float(self.get_parameter("row_y_tolerance_px").value)

        self.hand_detect_consecutive_frames = int(
            self.get_parameter("hand_detect_consecutive_frames").value
        )
        self.hand_lost_consecutive_frames = int(
            self.get_parameter("hand_lost_consecutive_frames").value
        )
        # end

        self.conf_threshold = float(self.get_parameter("confidence_threshold").value)
        self.num_tubes = int(self.get_parameter("num_tubes").value)

        # 260624 jiwan
        # self.tube_height_mm = float(self.get_parameter("tube_height_mm").value)
        # end

        self.hand_safety_enabled = bool(self.get_parameter("hand_safety_enabled").value)
        self.camera_timeout_sec = float(self.get_parameter("camera_timeout_sec").value)
        self.device = self.get_parameter("device").value

        model_path = self.get_parameter("model_path").value
        if not model_path:
            raise RuntimeError("model_path parameter is required (path to YOLOv8n .pt weights)")

        from ultralytics import YOLO  # imported lazily so node start-up errors are clear

        self.get_logger().info(f"Loading YOLO model from {model_path} (device={self.device})")
        self.model = YOLO(model_path)
        self.class_names = self.model.names  # e.g. {0: 'cup', 1: 'height', 2: 'hand'}

        self.last_image_time = None
        
        # 260624 jiwan buffer 초기화
        self.height_buffers = [
            deque(maxlen=self.height_buffer_size)
            for _ in range(self.num_tubes)
        ]

        self.conf_buffers = [
            deque(maxlen=self.height_buffer_size)
            for _ in range(self.num_tubes)
        ]

        self.hand_detect_count = 0
        self.hand_lost_count = 0
        self.hand_detected_state = False
        self.inference_count = 0 # 디버깅용 추론 시간 확인용
        # end

        # 260624 jiwan slot anchor: cup이 num_tubes개 동시에 보이는 첫 순간에
        # 한 번 부트스트랩해서 들고 있음. 폐기로 컵이 줄어도 이 anchor는 안 바뀌어서
        # 남은 컵들의 tube_index가 안 밀림. 트레이가 바뀌면 reset_slot_anchors_callback으로
        # None으로 되돌려서 다음 3개가 보일 때 다시 부트스트랩되게 함 (지금은 수동 훅).
        self.slot_anchors = None
        # end

        self.tube_height_pub = self.create_publisher(TubeHeight, "/vision/tube_height", 10)
        self.hand_detected_pub = self.create_publisher(Bool, "/vision/hand_detected", 10)
        self.camera_status_pub = self.create_publisher(Bool, "/vision/camera_status", 10)
        # 2026-06-25 soo: YOLO 추론 결과 이미지 발행 (HMI videoLabel에서 표시)
        self.yolo_image_pub = self.create_publisher(Image, "/vision/yolo_image", 10)

        # 2026-06-25 soo: HMI YOLO 토글 버튼 → 추론 on/off 구독
        self._yolo_enabled = True
        self.create_subscription(Bool, "/vision/yolo_enabled", self._on_yolo_enabled, 10)
        # 260624 jiwan QoS를 우리 입맛대로 수정
        # self.create_subscription(Image, "/vision/side_image", self.on_image, 10)
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # 260624 jiwan 구독 타입 Image -> CompressedImage
        self.create_subscription(
            CompressedImage,
            "/vision/side_image",
            self.on_image,
            image_qos
        )
        # end

        self.create_timer(0.5, self.check_camera_timeout)

        # 260624 jiwan 새 트레이로 바뀌었을 때 anchor를 다시 잡게 하는 수동 리셋 훅.
        # 나중에 "트레이 전체 이동 완료" 신호가 생기면 거기서 이 서비스를 호출하면 됨.
        self.create_service(Trigger, "/vision/reset_slot_anchors", self.handle_reset_slot_anchors)
        # end

        self.get_logger().info("liquid_height_detector_node ready")

    # 2026-06-25 soo: YOLO 추론 on/off — False 시 이미지 수신해도 추론 건너뜀
    def _on_yolo_enabled(self, msg: Bool):
        self._yolo_enabled = msg.data
        self.get_logger().info(f"YOLO inference {'ENABLED' if msg.data else 'DISABLED'}")

    def check_camera_timeout(self):
        if self.last_image_time is None:
            return
        elapsed = time.monotonic() - self.last_image_time
        camera_ok = elapsed <= self.camera_timeout_sec
        self.camera_status_pub.publish(Bool(data=camera_ok))
        if not camera_ok:
            self.get_logger().warn(
                f"No camera frames for {elapsed:.1f}s (timeout={self.camera_timeout_sec}s)"
            )

    # 260624 jiwan 새 트레이 전환 시 slot anchor + 버퍼 전부 초기화
    def handle_reset_slot_anchors(self, request, response):
        self.slot_anchors = None
        for idx in range(self.num_tubes):
            self.height_buffers[idx].clear()
            self.conf_buffers[idx].clear()
        response.success = True
        response.message = "slot_anchors reset; will re-bootstrap on next full sighting"
        self.get_logger().info(response.message)
        return response
    # end

    # 260624 jiwan msg 타입 Image -> CompressedImage, 디코드 함수도 교체
    def on_image(self, msg: CompressedImage):
        self.last_image_time = time.monotonic()

        # 2026-06-25 soo: YOLO 비활성화 시 추론 건너뜀 (카메라 타임아웃 체크는 유지)
        if not self._yolo_enabled:
            return

        frame = compressed_image_to_bgr8(msg)
        # end
        frame_height, frame_width = frame.shape[:2]

        # 260624 jiwan results 포함 내용 수정
        # results = self.model.predict(
        #     frame, conf=self.conf_threshold, device=self.device, verbose=False
        # )
        results = self.model.predict(
            frame,
            conf=self.conf_threshold,
            iou=self.yolo_iou_threshold,
            imgsz=self.yolo_imgsz,
            max_det=self.yolo_max_det,
            device=self.device,
            verbose=False
        )
        # end

        result = results[0]

        cup_boxes, height_boxes, hand_boxes = [], [], []
        for box in result.boxes:
            cls_id = int(box.cls[0])
            cls_name = self.class_names.get(cls_id, str(cls_id))
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = float(box.conf[0])
            b = Box(x1=x1, y1=y1, x2=x2, y2=y2, conf=conf)
            if cls_name == "cup":
                cup_boxes.append(b)
            elif cls_name == "height":
                height_boxes.append(b)
            elif cls_name == "hand":
                hand_boxes.append(b)
        # 260624 jiwan hand_detected 수정
        # if self.hand_safety_enabled:
        #     self.hand_detected_pub.publish(Bool(data=len(hand_boxes) > 0))
        hand_seen_now = len(hand_boxes) > 0

        if self.hand_safety_enabled:
            if hand_seen_now:
                self.hand_detect_count += 1
                self.hand_lost_count = 0
            else:
                self.hand_lost_count += 1
                self.hand_detect_count = 0

            if self.hand_detect_count >= self.hand_detect_consecutive_frames:
                self.hand_detected_state = True

            if self.hand_lost_count >= self.hand_lost_consecutive_frames:
                self.hand_detected_state = False

            self.hand_detected_pub.publish(Bool(data=self.hand_detected_state))

        # end
            
        # 260624 jiwan tube zone/rank 함수 말고 동적 anchor 부트스트랩 + 매칭으로 교체.
        # cup이 num_tubes개 동시에 보이는 첫 순간에만 anchor를 잡고, 그 뒤로는 폐기로
        # 컵이 줄어도 anchor를 그대로 유지해서 남은 컵들의 tube_index가 안 밀리게 함.
        if self.slot_anchors is None:
            confident_cups = [b for b in cup_boxes if b.conf >= self.conf_threshold]
            if len(confident_cups) >= self.num_tubes:
                # 한 프레임에 num_tubes보다 많이 잡히면(노이즈/뒷줄) y2(박스 하단,
                # 카메라에 가까울수록 큼)가 큰 것부터 앞줄로 보고 num_tubes개만 채택
                front = sorted(confident_cups, key=lambda b: b.y2, reverse=True)[: self.num_tubes]
                self.slot_anchors = sorted([(b.cx, b.y1) for b in front], key=lambda a: a[0])
                self.get_logger().info(f"slot_anchors bootstrapped: {self.slot_anchors}")

        if self.slot_anchors is None:
            tube_slots = [None] * self.num_tubes
        else:
            tube_slots = match_cups_to_anchors(
                cup_boxes, self.slot_anchors, self.slot_x_tolerance_px, self.row_y_tolerance_px
            )
        # end

        tube_index, liquid_height, confidence = [], [], []
        for idx, cup_box in enumerate(tube_slots):
            tube_index.append(idx)
            if cup_box is None:
                liquid_height.append(0.0)
                confidence.append(0.0)
                # 260624 jiwan 슬롯에 컵이 없으면 이전/다른 줄 값이 섞여 들어오지 않게 버퍼도 비움
                self.height_buffers[idx].clear()
                self.conf_buffers[idx].clear()
                continue

            height_box = match_height_box(cup_box, height_boxes)
            if height_box is None:
                liquid_height.append(0.0)
                confidence.append(0.0)
                self.height_buffers[idx].clear()
                self.conf_buffers[idx].clear()
                continue
            
            # 260624 jiwan height 계산 방식 수정 mm -> persent %
            # fraction = liquid_fill_fraction(cup_box, height_box)
            # tube_conf = min(cup_box.conf, height_box.conf)
            # liquid_height.append(fraction * self.tube_height_mm)
            # confidence.append(tube_conf)
            fraction = liquid_fill_fraction(cup_box, height_box)
            percent = fraction * 100.0
            tube_conf = min(cup_box.conf, height_box.conf)

            self.height_buffers[idx].append(percent)
            self.conf_buffers[idx].append(tube_conf)

            if len(self.height_buffers[idx]) >= self.height_publish_min_samples:
                if self.height_filter_method == "mean":
                    filtered_height = sum(self.height_buffers[idx]) / len(self.height_buffers[idx])
                    filtered_conf = sum(self.conf_buffers[idx]) / len(self.conf_buffers[idx])
                else:
                    filtered_height = statistics.median(self.height_buffers[idx])
                    filtered_conf = statistics.median(self.conf_buffers[idx])
            else:
                filtered_height = percent
                filtered_conf = tube_conf

            liquid_height.append(float(filtered_height))
            confidence.append(float(filtered_conf)) 

            # end


        msg_out = TubeHeight()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = "side_camera"
        msg_out.tube_index = tube_index
        msg_out.liquid_height = liquid_height
        msg_out.confidence = confidence
        self.tube_height_pub.publish(msg_out)

        # 2026-06-25 soo: YOLO result.plot()으로 바운딩박스 그린 이미지 받아서 발행
        annotated = result.plot()
        yolo_msg = bgr8_to_image(annotated)
        yolo_msg.header.stamp = msg_out.header.stamp
        self.yolo_image_pub.publish(yolo_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LiquidHeightDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
