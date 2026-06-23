"""Minimal sensor_msgs/Image <-> numpy bgr8 conversion.

cv_bridge's compiled extension is built against an older numpy ABI and
segfaults under numpy>=2 in this environment. Since every node in this
package only ever produces/consumes bgr8 frames, a manual conversion avoids
the dependency entirely.
"""
import numpy as np
from sensor_msgs.msg import Image


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
