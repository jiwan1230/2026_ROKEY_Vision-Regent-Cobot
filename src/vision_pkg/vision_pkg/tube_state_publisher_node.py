"""Classifies each tube's inferred liquid height into State 0/1/2.

Threshold logic follows the spec doc section 7 exactly:

    if refill_min <= height < target_height - tolerance: state = 0  # refill
    elif target_height - tolerance <= height <= target_height + tolerance: state = 1  # normal
    else: state = 2  # dispose

A tube whose detection confidence is below confidence_threshold is reported
as STATE_UNKNOWN (-1) instead, per the section 16 vision error-handling table
("시약통 검출 실패 -> UNKNOWN 표시").

Also hosts /vision/request_recheck (spec doc section 12): the robot calls
this after a refill/dispose action so the next freshly-classified
/vision/tube_state can be reported back through the service response.
"""
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from interfaces.msg import TubeHeight, TubeState
from interfaces.srv import RequestRecheck


class TubeStatePublisherNode(Node):
    def __init__(self):
        super().__init__("tube_state_publisher_node")

        self.declare_parameter("target_height", 60.0)
        self.declare_parameter("tolerance", 10.0)
        self.declare_parameter("refill_min", 20.0)
        self.declare_parameter("overflow_max", 95.0)
        self.declare_parameter("confidence_threshold", 0.4)
        self.declare_parameter("recheck_wait_timeout_sec", 5.0)

        self.target_height = float(self.get_parameter("target_height").value)
        self.tolerance = float(self.get_parameter("tolerance").value)
        self.refill_min = float(self.get_parameter("refill_min").value)
        self.overflow_max = float(self.get_parameter("overflow_max").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.recheck_wait_timeout_sec = float(self.get_parameter("recheck_wait_timeout_sec").value)

        self.last_state_msg = None
        self.last_publish_monotonic = 0.0

        cb_group = ReentrantCallbackGroup()
        self.state_pub = self.create_publisher(TubeState, "/vision/tube_state", 10)
        self.create_subscription(
            TubeHeight, "/vision/tube_height", self.on_tube_height, 10, callback_group=cb_group
        )
        self.create_service(
            RequestRecheck, "/vision/request_recheck", self.handle_request_recheck,
            callback_group=cb_group,
        )

        self.get_logger().info(
            "tube_state_publisher_node ready "
            f"(target={self.target_height}, tolerance={self.tolerance}, "
            f"refill_min={self.refill_min}, overflow_max={self.overflow_max})"
        )

    def classify(self, height, confidence):
        if confidence < self.confidence_threshold:
            return TubeState.STATE_UNKNOWN
        if self.refill_min <= height < self.target_height - self.tolerance:
            return TubeState.STATE_REFILL_NEEDED
        if self.target_height - self.tolerance <= height <= self.target_height + self.tolerance:
            return TubeState.STATE_NORMAL
        return TubeState.STATE_DISPOSE_NEEDED

    def on_tube_height(self, msg: TubeHeight):
        states = [
            self.classify(h, c) for h, c in zip(msg.liquid_height, msg.confidence)
        ]

        out = TubeState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.tube_index = list(msg.tube_index)
        out.state = states
        self.state_pub.publish(out)
        self.last_state_msg = out
        self.last_publish_monotonic = time.monotonic()

        self.get_logger().debug(f"tube_index={out.tube_index} state={out.state}")

    def handle_request_recheck(self, request, response):
        if not request.request:
            response.success = True
            response.message = "no-op"
            return response

        start = time.monotonic()
        target_after = start
        while time.monotonic() - start < self.recheck_wait_timeout_sec:
            if self.last_publish_monotonic > target_after:
                response.success = True
                response.message = f"recheck complete: state={list(self.last_state_msg.state)}"
                return response
            time.sleep(0.05)

        response.success = False
        response.message = "recheck timed out waiting for fresh tube_state"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TubeStatePublisherNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
