"""Minimal sensor_msgs/Image <-> numpy bgr8 conversion.

cv_bridge's compiled extension is built against an older numpy ABI and
segfaults under numpy>=2 in this environment. Since every node in this
package only ever produces/consumes bgr8 frames, a manual conversion avoids
the dependency entirely.
"""
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image


def image_to_bgr8(msg: Image) -> np.ndarray:
    if msg.encoding != "bgr8":
        raise ValueError(f"Unsupported encoding: {msg.encoding} (expected bgr8)")
    return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)


def bgr8_to_image(frame: np.ndarray) -> Image:
    msg = Image()
    msg.height, msg.width = frame.shape[0], frame.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(frame).tobytes()
    return msg


# 260624 jiwan raw bgr8(640x480=921KB)가 BEST_EFFORT QoS에서 UDP 단편화로 드롭되어
# side_image가 끊기는 문제 발견 -> JPEG로 압축해서 보내고 받는 쪽에서 디코드
def bgr8_to_compressed_image(frame: np.ndarray, quality: int = 90) -> CompressedImage:
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError("JPEG encode failed")
    msg = CompressedImage()
    msg.format = "jpeg"
    msg.data = encoded.tobytes()
    return msg


def compressed_image_to_bgr8(msg: CompressedImage) -> np.ndarray:
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("JPEG decode failed")
    return frame
# end
