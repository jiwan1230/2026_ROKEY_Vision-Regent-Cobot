"""Gripper control node for OnRobot RG2 via Doosan digital I/O + Modbus TCP.

Commands the gripper with set_digital_output (existing wiring).
Waits for motion-done by polling the RG2 status register (268) over
Modbus TCP at 192.168.1.1:502 — no pymodbus dependency needed.

Register 268 status bits (from OnRobot RG protocol spec):
  bit 0: busy         — 1 while motion ongoing, 0 when done
  bit 1: grip_detected — 1 when object gripped
"""
import socket
import struct
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from interfaces.srv import GripperControl

if not rclpy.ok():
    rclpy.init(args=sys.argv)

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

dsr_node = rclpy.create_node('dsr_lib_node', namespace=ROBOT_ID)

import DR_init
DR_init.__dsr__node = dsr_node
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from DSR_ROBOT2 import set_digital_output, ON, OFF

_MODBUS_IP   = "192.168.1.1"
_MODBUS_PORT = 502
_MODBUS_UNIT = 65
_STATUS_REG  = 268


class GripperControlNode(Node):
    def __init__(self):
        super().__init__("gripper_control_node")
        self.gripper_state = "OPEN"
        self._modbus_sock = None
        self._connect_modbus()
        self.create_service(GripperControl, "/gripper/control", self.handle_gripper_control)
        self.get_logger().info("gripper_control_node ready")

    # ------------------------------------------------------------------
    # Modbus TCP (raw socket, no external dependency)
    # ------------------------------------------------------------------

    def _connect_modbus(self):
        try:
            sock = socket.create_connection((_MODBUS_IP, _MODBUS_PORT), timeout=2.0)
            sock.settimeout(1.0)
            self._modbus_sock = sock
            self.get_logger().info(f"Modbus TCP connected: {_MODBUS_IP}:{_MODBUS_PORT}")
        except Exception as e:
            self._modbus_sock = None
            self.get_logger().warn(f"Modbus TCP connect failed: {e} — will retry on next grip")

    def _read_status_register(self):
        """Return register 268 value, or None on error (with auto-reconnect)."""
        for _ in range(2):
            if self._modbus_sock is None:
                self._connect_modbus()
            if self._modbus_sock is None:
                return None
            try:
                # Modbus TCP read holding registers: TxID=1, Proto=0, Len=6, Unit, FC=3, Addr, Count=1
                req = struct.pack('>HHHBBHH', 1, 0, 6, _MODBUS_UNIT, 3, _STATUS_REG, 1)
                self._modbus_sock.sendall(req)
                resp = self._modbus_sock.recv(256)
                if len(resp) >= 11:
                    return struct.unpack('>H', resp[9:11])[0]
            except Exception:
                self._modbus_sock = None
        return None

    def _wait_grip_done(self, timeout_sec=3.0):
        """Block until gripper busy-bit clears (motion done) or timeout."""
        start = time.time()
        while time.time() - start < timeout_sec:
            status = self._read_status_register()
            if status is not None:
                time.sleep(0.3)
                busy = status & 0x01
                if not busy:
                    elapsed = time.time() - start
                    self.get_logger().info(f"Grip done in {elapsed:.2f}s (status=0x{status:04X})")
                    return True
            time.sleep(0.01)
        self.get_logger().warn(f"_wait_grip_done timed out after {timeout_sec}s")
        return False

    # ------------------------------------------------------------------
    # Service handler
    # ------------------------------------------------------------------

    def handle_gripper_control(self, request, response):
        command = request.command
        if command == GripperControl.Request.COMMAND_OPEN:
            self.open_gripper()
        elif command == GripperControl.Request.COMMAND_CLOSE:
            self.close_gripper()
        else:
            response.success = False
            response.message = f"Unsupported gripper command: {command}"
            self.get_logger().error(response.message)
            return response
        self.gripper_state = command
        response.success = True
        response.message = f"Gripper {command}"
        return response

    def open_gripper(self):
        self.get_logger().info("그리퍼 열기")
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        self._wait_grip_done(timeout_sec=3.0)

    def close_gripper(self):
        self.get_logger().info("그리퍼 닫기")
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        self._wait_grip_done(timeout_sec=3.0)


def main(args=None):
    gripper_node = GripperControlNode()

    executor = MultiThreadedExecutor()
    executor.add_node(gripper_node)
    executor.add_node(dsr_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if gripper_node._modbus_sock:
            gripper_node._modbus_sock.close()
        gripper_node.destroy_node()
        dsr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
