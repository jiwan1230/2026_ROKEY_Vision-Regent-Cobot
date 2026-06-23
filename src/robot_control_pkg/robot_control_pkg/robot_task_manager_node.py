"""Orchestrates refill / dispose / normal-transfer task sequences.

Hosts /robot/start_task and /robot/stop_task. Each start_task call handles
ONE priority tier per the spec doc's Scenario D rule
(State2 dispose > State0 refill > State1 normal-transfer) and then asks
vision for a recheck. This mirrors the section-14 flowchart exactly: a
recheck loops back to image Capture rather than being handled inline here,
so the *next* /vision/tube_state update (driven by main_decision_node) is
what decides the following action. Multi-tier requests therefore resolve
over a few reactive cycles instead of one long blocking call.
"""
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from interfaces.msg import RobotStatus, TubeState
from interfaces.srv import GripperControl, MoveToPose, RequestRecheck, StartTask, StopTask
from robot_control_pkg.poses import (
    HOME_POSE,
    NORMAL_TRAY_APPROACH_POSE,
    NORMAL_TRAY_POSE,
    TRAY_PICK_POSE,
    WASTE_POSE,
    tube_approach_pose,
    tube_pick_pose,
    tube_refill_pose,
)


class TaskAborted(Exception):
    """Raised internally when an emergency stop interrupts a task sequence."""


class TaskFailed(Exception):
    """Raised internally when a motion/gripper step reports failure."""


class RobotTaskManagerNode(Node):
    def __init__(self):
        super().__init__("robot_task_manager_node")

        self.declare_parameter("grip_retry_count", 1)
        self.declare_parameter("recheck_timeout_sec", 5.0)
        self.declare_parameter("service_call_timeout_sec", 10.0)

        self.grip_retry_count = int(self.get_parameter("grip_retry_count").value)
        self.recheck_timeout_sec = float(self.get_parameter("recheck_timeout_sec").value)
        self.service_call_timeout_sec = float(self.get_parameter("service_call_timeout_sec").value)

        self.stop_event = threading.Event()
        cb_group = ReentrantCallbackGroup()

        self.status_pub = self.create_publisher(RobotStatus, "/robot/status", 10)

        self.move_client = self.create_client(
            MoveToPose, "/robot/move_to_pose", callback_group=cb_group
        )
        self.gripper_client = self.create_client(
            GripperControl, "/gripper/control", callback_group=cb_group
        )
        self.recheck_client = self.create_client(
            RequestRecheck, "/vision/request_recheck", callback_group=cb_group
        )

        self.create_service(
            StartTask, "/robot/start_task", self.handle_start_task, callback_group=cb_group
        )
        self.create_service(
            StopTask, "/robot/stop_task", self.handle_stop_task, callback_group=cb_group
        )

        self.publish_status(RobotStatus.STATUS_IDLE, detail="robot_task_manager_node ready")
        self.get_logger().info("robot_task_manager_node ready")

    # ---- low-level helpers -------------------------------------------------

    def publish_status(self, status, current_task="", detail=""):
        msg = RobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = status
        msg.current_task = current_task
        msg.detail = detail
        self.status_pub.publish(msg)

    def _call_sync(self, client, request, timeout_sec=None):
        timeout_sec = timeout_sec or self.service_call_timeout_sec
        if not client.wait_for_service(timeout_sec=2.0):
            raise TaskFailed(f"Service {client.srv_name} unavailable")
        future = client.call_async(request)
        start = time.monotonic()
        while not future.done():
            if time.monotonic() - start > timeout_sec:
                raise TaskFailed(f"Service {client.srv_name} timed out")
            time.sleep(0.01)
        return future.result()

    def _check_stop(self):
        if self.stop_event.is_set():
            raise TaskAborted()

    def move(self, pose_name):
        self._check_stop()
        result = self._call_sync(self.move_client, MoveToPose.Request(pose_name=pose_name))
        if not result.success:
            raise TaskFailed(result.message)

    def grip(self, command):
        result = self._call_sync(self.gripper_client, GripperControl.Request(command=command))
        return result.success

    def grip_with_retry(self, command):
        for attempt in range(self.grip_retry_count + 1):
            self._check_stop()
            if self.grip(command):
                return True
            self.get_logger().warn(f"Gripper {command} failed, attempt {attempt + 1}")
        return False

    def request_recheck(self):
        try:
            result = self._call_sync(
                self.recheck_client, RequestRecheck.Request(request=True), self.recheck_timeout_sec
            )
            self.get_logger().info(f"Recheck requested: {result.message}")
        except TaskFailed as e:
            self.get_logger().warn(f"Recheck request failed: {e}")

    # ---- task sequences -----------------------------------------------------

    def dispose_tube(self, idx):
        self.publish_status(RobotStatus.STATUS_MOVING, "dispose", f"approach tube {idx}")
        self.move(tube_approach_pose(idx))
        self.move(tube_pick_pose(idx))

        self.publish_status(RobotStatus.STATUS_PICKING, "dispose", f"grip tube {idx}")
        if not self.grip_with_retry(GripperControl.Request.COMMAND_CLOSE):
            raise TaskFailed(f"Gripper failed to close on tube {idx}")

        self.move(tube_approach_pose(idx))
        self.publish_status(RobotStatus.STATUS_DISPOSING, "dispose", f"move tube {idx} to waste zone")
        self.move(WASTE_POSE)
        self.grip(GripperControl.Request.COMMAND_OPEN)
        self.move(HOME_POSE)

    def refill_tube(self, idx):
        self.publish_status(RobotStatus.STATUS_MOVING, "refill", f"approach tube {idx}")
        self.move(tube_approach_pose(idx))

        self.publish_status(RobotStatus.STATUS_REFILLING, "refill", f"dispense reagent into tube {idx}")
        self.move(tube_refill_pose(idx))
        self._check_stop()
        time.sleep(0.3)  # simulated dispense duration for the configured fill amount

        self.move(tube_approach_pose(idx))
        self.move(HOME_POSE)

    def transfer_tray(self):
        self.publish_status(RobotStatus.STATUS_MOVING, "transfer_normal", "approach tray")
        self.move(TRAY_PICK_POSE)

        self.publish_status(RobotStatus.STATUS_PICKING, "transfer_normal", "grip tray")
        if not self.grip_with_retry(GripperControl.Request.COMMAND_CLOSE):
            raise TaskFailed("Gripper failed to grip tray")

        self.publish_status(
            RobotStatus.STATUS_TRANSFER_NORMAL, "transfer_normal", "move tray to normal zone"
        )
        self.move(NORMAL_TRAY_APPROACH_POSE)
        self.move(NORMAL_TRAY_POSE)
        self.grip(GripperControl.Request.COMMAND_OPEN)
        self.move(HOME_POSE)

    # ---- service handlers ---------------------------------------------------

    def handle_start_task(self, request, response):
        self.stop_event.clear()
        state_map = dict(zip(request.tube_index, request.state))

        dispose_idxs = sorted(i for i, s in state_map.items() if s == TubeState.STATE_DISPOSE_NEEDED)
        refill_idxs = sorted(i for i, s in state_map.items() if s == TubeState.STATE_REFILL_NEEDED)
        all_normal = bool(state_map) and all(s == TubeState.STATE_NORMAL for s in state_map.values())

        try:
            if dispose_idxs:
                for idx in dispose_idxs:
                    self.dispose_tube(idx)
                self.request_recheck()
                response.success = True
                response.message = f"Disposed tube(s) {dispose_idxs}; recheck requested"
            elif refill_idxs:
                for idx in refill_idxs:
                    self.refill_tube(idx)
                self.request_recheck()
                response.success = True
                response.message = f"Refilled tube(s) {refill_idxs}; recheck requested"
            elif all_normal:
                self.transfer_tray()
                response.success = True
                response.message = "Tray transferred to normal zone"
            else:
                response.success = True
                response.message = "No action required"

            self.publish_status(RobotStatus.STATUS_IDLE, detail=response.message)

        except TaskAborted:
            response.success = False
            response.message = "Task aborted by emergency stop"
            self.publish_status(RobotStatus.STATUS_EMERGENCY_STOP, detail=response.message)
        except TaskFailed as e:
            response.success = False
            response.message = str(e)
            self.publish_status(RobotStatus.STATUS_ERROR, detail=response.message)

        return response

    def handle_stop_task(self, request, response):
        if request.stop:
            self.stop_event.set()
            response.success = True
            response.message = "Stop requested; current task will abort at the next safe checkpoint"
            self.get_logger().warn("Emergency stop requested")
        else:
            response.success = True
            response.message = "No-op (stop=false)"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotTaskManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
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
