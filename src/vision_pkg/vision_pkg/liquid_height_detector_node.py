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
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from interfaces.msg import TubeHeight
from vision_pkg.bbox_utils import Box, assign_tube_zones, liquid_fill_fraction, match_height_box
from vision_pkg.ros_image_utils import image_to_bgr8


class LiquidHeightDetectorNode(Node):
    def __init__(self):
        super().__init__("liquid_height_detector_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("confidence_threshold", 0.4)
        self.declare_parameter("num_tubes", 3)
        self.declare_parameter("tube_height_mm", 100.0)
        self.declare_parameter("hand_safety_enabled", True)
        self.declare_parameter("camera_timeout_sec", 3.0)

        self.conf_threshold = float(self.get_parameter("confidence_threshold").value)
        self.num_tubes = int(self.get_parameter("num_tubes").value)
        self.tube_height_mm = float(self.get_parameter("tube_height_mm").value)
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

        self.tube_height_pub = self.create_publisher(TubeHeight, "/vision/tube_height", 10)
        self.hand_detected_pub = self.create_publisher(Bool, "/vision/hand_detected", 10)
        self.camera_status_pub = self.create_publisher(Bool, "/vision/camera_status", 10)

        self.create_subscription(Image, "/vision/side_image", self.on_image, 10)
        self.create_timer(0.5, self.check_camera_timeout)

        self.get_logger().info("liquid_height_detector_node ready")

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

    def on_image(self, msg: Image):
        self.last_image_time = time.monotonic()

        frame = image_to_bgr8(msg)
        frame_height, frame_width = frame.shape[:2]

        results = self.model.predict(
            frame, conf=self.conf_threshold, device=self.device, verbose=False
        )
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

        if self.hand_safety_enabled:
            self.hand_detected_pub.publish(Bool(data=len(hand_boxes) > 0))

        tube_slots = assign_tube_zones(cup_boxes, self.num_tubes, frame_width)

        tube_index, liquid_height, confidence = [], [], []
        for idx, cup_box in enumerate(tube_slots):
            tube_index.append(idx)
            if cup_box is None:
                liquid_height.append(0.0)
                confidence.append(0.0)
                continue

            height_box = match_height_box(cup_box, height_boxes)
            if height_box is None:
                liquid_height.append(0.0)
                confidence.append(0.0)
                continue

            fraction = liquid_fill_fraction(cup_box, height_box)
            tube_conf = min(cup_box.conf, height_box.conf)
            liquid_height.append(fraction * self.tube_height_mm)
            confidence.append(tube_conf)

        msg_out = TubeHeight()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = "side_camera"
        msg_out.tube_index = tube_index
        msg_out.liquid_height = liquid_height
        msg_out.confidence = confidence
        self.tube_height_pub.publish(msg_out)


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
