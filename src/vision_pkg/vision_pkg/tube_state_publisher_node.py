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
# 260624 jiwan 폐기 완료 알림(handled_slots override) + 새 트레이 수동 리셋용
from interfaces.srv import MarkTubeDisposed
from std_srvs.srv import Trigger
# end
# 260624 jiwan refill 루프가 매번 조회하는 per-tube 현재 상태 서비스 (지금까지
# 서버가 없어서 robot_task_manager_node에서 호출하면 죽는 상태였음)
from interfaces.srv import CurrentTubeState
# end


class TubeStatePublisherNode(Node):
    def __init__(self):
        super().__init__("tube_state_publisher_node")

        self.declare_parameter("target_height", 60.0)
        self.declare_parameter("tolerance", 10.0)
        self.declare_parameter("refill_min", 20.0)
        self.declare_parameter("overflow_max", 95.0)
        self.declare_parameter("confidence_threshold", 0.4)
        self.declare_parameter("recheck_wait_timeout_sec", 5.0)
        # 260624 jiwan handled_slots 배열 크기에 필요
        self.declare_parameter("num_tubes", 3)
        # end

        self.target_height = float(self.get_parameter("target_height").value)
        self.tolerance = float(self.get_parameter("tolerance").value)
        self.refill_min = float(self.get_parameter("refill_min").value)
        self.overflow_max = float(self.get_parameter("overflow_max").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.recheck_wait_timeout_sec = float(self.get_parameter("recheck_wait_timeout_sec").value)
        self.num_tubes = int(self.get_parameter("num_tubes").value)

        self.last_state_msg = None
        self.last_height_msg = None
        self.last_publish_monotonic = 0.0

        # 260624 jiwan 로봇이 "이 tube 폐기했다"고 알려준 슬롯은 vision이 안 보여도
        # NORMAL로 취급. vision 단독으론 안 보이는 이유(폐기/오검출/가림/뒷줄겹침)를
        # 구분할 수 없어서, 폐기는 로봇이 직접 알려주는 신호를 신뢰함.
        self.handled_slots = [False] * self.num_tubes
        # end

        cb_group = ReentrantCallbackGroup()
        self.state_pub = self.create_publisher(TubeState, "/vision/tube_state", 10)
        self.create_subscription(
            TubeHeight, "/vision/tube_height", self.on_tube_height, 10, callback_group=cb_group
        )
        self.create_service(
            RequestRecheck, "/vision/request_recheck", self.handle_request_recheck,
            callback_group=cb_group,
        )
        # 260624 jiwan 폐기 완료 알림 + 새 트레이 전환 시 handled_slots 수동 리셋
        self.create_service(
            MarkTubeDisposed, "/vision/mark_tube_disposed", self.handle_mark_tube_disposed,
            callback_group=cb_group,
        )
        self.create_service(
            Trigger, "/vision/reset_handled_slots", self.handle_reset_handled_slots,
            callback_group=cb_group,
        )
        # end
        # 260624 jiwan refill 루프 중간에 조회하는 per-tube 현재 상태 서비스
        self.create_service(
            CurrentTubeState, "/robot/current_tube_state", self.handle_current_tube_state,
            callback_group=cb_group,
        )
        # end

        self.get_logger().info(
            "tube_state_publisher_node ready "
            f"(target={self.target_height}, tolerance={self.tolerance}, "
            f"refill_min={self.refill_min}, overflow_max={self.overflow_max})"
        )

    def classify(self, idx, height, confidence):
        # 260624 jiwan 폐기 완료로 알려진 슬롯은 vision 검출과 무관하게 NORMAL 취급
        if self.handled_slots[idx]:
            return TubeState.STATE_NORMAL
        # end
        if confidence < self.confidence_threshold:
            return TubeState.STATE_UNKNOWN
        if self.refill_min <= height < self.target_height - self.tolerance:
            return TubeState.STATE_REFILL_NEEDED
        if self.target_height - self.tolerance <= height <= self.target_height + self.tolerance:
            return TubeState.STATE_NORMAL
        return TubeState.STATE_DISPOSE_NEEDED

    def on_tube_height(self, msg: TubeHeight):
        self.last_height_msg = msg  # 260624 jiwan CurrentTubeState 서비스가 조회할 원본 데이터
        states = [
            self.classify(idx, h, c)
            for idx, h, c in zip(msg.tube_index, msg.liquid_height, msg.confidence)
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

    # 260624 jiwan 로봇이 폐기 완료한 tube_index를 알려주면 handled_slots override
    def handle_mark_tube_disposed(self, request, response):
        idx = request.tube_index
        if not (0 <= idx < self.num_tubes):
            response.success = False
            response.message = f"tube_index {idx} out of range (num_tubes={self.num_tubes})"
            return response

        self.handled_slots[idx] = True
        response.success = True
        response.message = f"tube {idx} marked disposed"
        self.get_logger().info(response.message)
        return response

    # 260624 jiwan 새 트레이로 바뀌었을 때 handled_slots 전부 초기화 (수동 훅,
    # 나중에 "트레이 전체 이동 완료" 신호가 생기면 거기서 호출하면 됨)
    def handle_reset_handled_slots(self, request, response):
        self.handled_slots = [False] * self.num_tubes
        response.success = True
        response.message = "handled_slots reset"
        self.get_logger().info(response.message)
        return response
    # end

    # 260624 jiwan refill 루프가 "지금 이 tube 상태가 NORMAL인지" 매 iteration마다
    # 묻는 서비스. current_ratio는 0~1 비율로 변환해서 반환 (liquid_height/target_height
    # 등은 0~100 percent라서 doosan move()의 rotate 계산과 스케일이 다름).
    def handle_current_tube_state(self, request, response):
        idx = request.tube_index
        if not (0 <= idx < self.num_tubes):
            response.success = False
            response.message = f"tube_index {idx} out of range (num_tubes={self.num_tubes})"
            return response
        if self.last_height_msg is None or idx not in list(self.last_height_msg.tube_index):
            response.success = False
            response.message = f"no tube_height data yet for tube {idx}"
            return response

        pos = list(self.last_height_msg.tube_index).index(idx)
        height_percent = self.last_height_msg.liquid_height[pos]
        confidence = self.last_height_msg.confidence[pos]

        response.success = True
        response.state = self.classify(idx, height_percent, confidence)
        response.current_ratio = height_percent / 100.0
        response.confidence = confidence
        response.message = f"tube {idx}: state={response.state} ratio={response.current_ratio:.2f}"
        return response
    # end


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
