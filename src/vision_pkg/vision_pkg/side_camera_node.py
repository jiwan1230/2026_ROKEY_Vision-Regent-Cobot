"""Side-view camera capture node.

Captures frames from a USB/web camera (or, for development without
physical hardware, loops a video file or a directory of still images)
and publishes them as sensor_msgs/Image on /vision/side_image.
"""
import glob
import os

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from vision_pkg.ros_image_utils import bgr8_to_image


class SideCameraNode(Node):
    def __init__(self):
        super().__init__("side_camera_node")

        self.declare_parameter("source_mode", "device")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("video_path", "")
        self.declare_parameter("image_dir", "")
        self.declare_parameter("image_loop", True)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("frame_width", 1280)
        self.declare_parameter("frame_height", 720)

        self.source_mode = self.get_parameter("source_mode").value
        self.image_loop = self.get_parameter("image_loop").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.publisher = self.create_publisher(Image, "/vision/side_image", 10)

        self.cap = None
        self.image_files = []
        self.image_file_idx = 0

        if self.source_mode == "device":
            camera_index = int(self.get_parameter("camera_index").value)
            self.cap = cv2.VideoCapture(camera_index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("frame_width").value))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("frame_height").value))
            if not self.cap.isOpened():
                self.get_logger().error(f"Failed to open camera device index {camera_index}")
        elif self.source_mode == "video_file":
            video_path = self.get_parameter("video_path").value
            self.cap = cv2.VideoCapture(video_path)
            if not self.cap.isOpened():
                self.get_logger().error(f"Failed to open video file {video_path}")
        elif self.source_mode == "image_dir":
            image_dir = self.get_parameter("image_dir").value
            patterns = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
            files = []
            for pattern in patterns:
                files.extend(glob.glob(os.path.join(image_dir, pattern)))
            self.image_files = sorted(files)
            if not self.image_files:
                self.get_logger().error(f"No images found in image_dir: {image_dir}")
        else:
            self.get_logger().error(f"Unknown source_mode: {self.source_mode}")

        period = 1.0 / publish_rate_hz if publish_rate_hz > 0 else 1.0
        self.timer = self.create_timer(period, self.publish_frame)
        self.get_logger().info(
            f"side_camera_node started (source_mode={self.source_mode}, rate={publish_rate_hz}Hz)"
        )

    def publish_frame(self):
        frame = None

        if self.source_mode in ("device", "video_file"):
            if self.cap is None or not self.cap.isOpened():
                return
            ok, frame = self.cap.read()
            if not ok:
                if self.source_mode == "video_file" and self.image_loop:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ok, frame = self.cap.read()
                if not ok:
                    self.get_logger().warn("Failed to read frame from camera/video source")
                    return
        elif self.source_mode == "image_dir":
            if not self.image_files:
                return
            path = self.image_files[self.image_file_idx]
            frame = cv2.imread(path)
            self.image_file_idx += 1
            if self.image_file_idx >= len(self.image_files):
                self.image_file_idx = 0 if self.image_loop else len(self.image_files) - 1
            if frame is None:
                self.get_logger().warn(f"Failed to read image file {path}")
                return

        if frame is None:
            return

        msg = bgr8_to_image(frame)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "side_camera"
        self.publisher.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SideCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
