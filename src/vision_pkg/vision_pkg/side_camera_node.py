"""Side-view camera capture node.

Captures frames from a USB/web camera (or, for development without
physical hardware, loops a video file or a directory of still images)
and publishes them as sensor_msgs/CompressedImage (JPEG) on /vision/side_image.
"""
import glob
import os

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

# 260624 jiwan side_image를 raw Image 대신 JPEG로 압축한 CompressedImage로 전송
from sensor_msgs.msg import CompressedImage
from vision_pkg.ros_image_utils import bgr8_to_compressed_image
# end

# 260624 jiwan import 추가
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# end


class SideCameraNode(Node):
    def __init__(self):
        super().__init__("side_camera_node")

        self.declare_parameter("source_mode", "device")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("video_path", "")
        self.declare_parameter("image_dir", "")
        self.declare_parameter("image_loop", True)
        # 260624 jiwan default 값 수정
        self.declare_parameter("publish_rate_hz", 7.0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        # end

        # 260624 jiwan JPEG 압축 품질 파라미터 추가
        self.declare_parameter("jpeg_quality", 90)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        # end

        self.source_mode = self.get_parameter("source_mode").value
        self.image_loop = self.get_parameter("image_loop").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        # 260624_jiwan input_image_size 조절(.yaml 파일에 파라미터 추가 및 불러오기)
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)   

        # self.publisher = self.create_publisher(Image, "/vision/side_image", 10)
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # 260624 jiwan publisher 타입 Image -> CompressedImage
        self.publisher = self.create_publisher(CompressedImage, "/vision/side_image", image_qos)
        # end
        # HMI Vision Log 탭용 - 카메라 연결 실패/해상도 변경 등을 사람이 읽을
        # 문장으로 같이 발행한다. get_logger()는 그대로 둠.
        self.vision_log_pub = self.create_publisher(String, "/vision/log", 10)

        self.cap = None
        self.image_files = []
        self.image_file_idx = 0

        if self.source_mode == "device":
            camera_index = int(self.get_parameter("camera_index").value)
            self.cap = cv2.VideoCapture(camera_index)
            # 260624_jiwan parameter 호출 방식 수정
            # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("frame_width").value))
            # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("frame_height").value))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, publish_rate_hz)
            # end

            if not self.cap.isOpened():
                self._log_event(f"Failed to open camera device index {camera_index}", level="error")
        elif self.source_mode == "video_file":
            video_path = self.get_parameter("video_path").value
            self.cap = cv2.VideoCapture(video_path)
            if not self.cap.isOpened():
                self._log_event(f"Failed to open video file {video_path}", level="error")
        elif self.source_mode == "image_dir":
            image_dir = self.get_parameter("image_dir").value
            patterns = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
            files = []
            for pattern in patterns:
                files.extend(glob.glob(os.path.join(image_dir, pattern)))
            self.image_files = sorted(files)
            if not self.image_files:
                self._log_event(f"No images found in image_dir: {image_dir}", level="error")
        else:
            self._log_event(f"Unknown source_mode: {self.source_mode}", level="error")

        # HMI 해상도 적용 버튼 - "WxH" 문자열로 들어옴 (예: "1280x720")
        self.create_subscription(String, "/camera/resolution_cmd", self.on_resolution_cmd, 10)

        period = 1.0 / publish_rate_hz if publish_rate_hz > 0 else 1.0
        self.timer = self.create_timer(period, self.publish_frame)
        self.get_logger().info(
            f"side_camera_node started (source_mode={self.source_mode}, rate={publish_rate_hz}Hz)"
        )
        # 2026-06-27 soo: HMI에서 런타임 FPS 변경 지원
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _on_set_parameters(self, params):
        # 2026-06-27 soo: publish_rate_hz 변경 시 타이머 재생성
        for p in params:
            if p.name == 'publish_rate_hz':
                rate = float(p.value.double_value)
                if rate > 0:
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / rate, self.publish_frame)
                    self._log_event(f"FPS 변경: {rate:.1f} Hz")
        return SetParametersResult(successful=True)

    def _log_event(self, message, level="info"):
        getattr(self.get_logger(), level)(message)
        self.vision_log_pub.publish(String(data=message))

    def on_resolution_cmd(self, msg: String):
        try:
            w_str, h_str = msg.data.lower().split("x")
            width, height = int(w_str), int(h_str)
        except ValueError:
            self._log_event(f"Invalid resolution_cmd: {msg.data!r} (expected 'WxH')", level="warn")
            return

        self.frame_width = width
        self.frame_height = height
        if self.source_mode == "device" and self.cap is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._log_event(f"Resolution changed to {width}x{height}")

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
        
        # 260624_jiwan publish 직전 이미지 resize
        # 카메라가 cap.set 해상도를 무시해도 최종 발행 이미지는 강제로 통일
        if self.frame_width > 0 and self.frame_height > 0:
            frame = cv2.resize(
                frame,
                (self.frame_width, self.frame_height),
                interpolation=cv2.INTER_AREA
            )
        # end

        # 260624 jiwan msg = bgr8_to_image(frame) -> JPEG로 압축해서 payload 크기 줄임
        msg = bgr8_to_compressed_image(frame, self.jpeg_quality)
        # end
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
