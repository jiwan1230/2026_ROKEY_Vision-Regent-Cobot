"""Operator HMI dashboard (spec doc section 15).

Panels: Camera View, Tube State Panel (per-tube height/state, color-coded
0=Yellow/1=Green/2=Red per the spec's color table), Robot Status Panel,
Control Panel (Start/Stop/Recheck/Emergency Stop), Log Panel, Safety Panel.

rclpy is spun on a background thread; Tkinter owns the main thread and
polls the cached node state via root.after(), which is the standard way to
bridge ROS callbacks into a Tkinter mainloop without cross-thread widget
access.
"""
import threading
import tkinter as tk
from datetime import datetime
from tkinter import ttk

import numpy as np
import rclpy
from PIL import Image as PILImage
from PIL import ImageTk
from rclpy.node import Node
from std_msgs.msg import Bool

from interfaces.msg import RobotStatus, TubeHeight, TubeState
from interfaces.srv import RequestRecheck, StartTask, StopTask

STATE_LABELS = {
    TubeState.STATE_REFILL_NEEDED: "REFILL NEEDED",
    TubeState.STATE_NORMAL: "NORMAL",
    TubeState.STATE_DISPOSE_NEEDED: "DISPOSE NEEDED",
    TubeState.STATE_UNKNOWN: "UNKNOWN",
}
STATE_COLORS = {
    TubeState.STATE_REFILL_NEEDED: "#e6c200",  # Yellow
    TubeState.STATE_NORMAL: "#2e8b2e",  # Green
    TubeState.STATE_DISPOSE_NEEDED: "#c0392b",  # Red
    TubeState.STATE_UNKNOWN: "#888888",  # Gray
}


def decode_bgr8(msg) -> np.ndarray:
    return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)


class HmiRosBridge(Node):
    """Holds all ROS interfaces; HmiApp polls/calls into this from Tk's
    main thread (attribute reads/writes only - cheap enough under the GIL)."""

    def __init__(self):
        super().__init__("hmi_node")

        self.latest_frame = None
        self.latest_tube_state = None
        self.latest_tube_height = None
        self.latest_robot_status = None
        self.camera_ok = True
        self.hand_detected = False

        from sensor_msgs.msg import Image

        self.create_subscription(Image, "/vision/side_image", self._on_image, 10)
        self.create_subscription(TubeState, "/vision/tube_state", self._on_tube_state, 10)
        self.create_subscription(TubeHeight, "/vision/tube_height", self._on_tube_height, 10)
        self.create_subscription(RobotStatus, "/robot/status", self._on_robot_status, 10)
        self.create_subscription(Bool, "/vision/camera_status", self._on_camera_status, 10)
        self.create_subscription(Bool, "/vision/hand_detected", self._on_hand_detected, 10)

        self.start_task_client = self.create_client(StartTask, "/robot/start_task")
        self.stop_task_client = self.create_client(StopTask, "/robot/stop_task")
        self.recheck_client = self.create_client(RequestRecheck, "/vision/request_recheck")

    def _on_image(self, msg):
        try:
            self.latest_frame = decode_bgr8(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to decode camera image: {e}")

    def _on_tube_state(self, msg):
        self.latest_tube_state = msg

    def _on_tube_height(self, msg):
        self.latest_tube_height = msg

    def _on_robot_status(self, msg):
        self.latest_robot_status = msg

    def _on_camera_status(self, msg):
        self.camera_ok = msg.data

    def _on_hand_detected(self, msg):
        self.hand_detected = msg.data

    def call_start_task(self):
        if self.latest_tube_state is None:
            return None
        req = StartTask.Request(
            tube_index=list(self.latest_tube_state.tube_index),
            state=list(self.latest_tube_state.state),
        )
        return self.start_task_client.call_async(req)

    def call_stop_task(self):
        return self.stop_task_client.call_async(StopTask.Request(stop=True))

    def call_request_recheck(self):
        return self.recheck_client.call_async(RequestRecheck.Request(request=True))


class HmiApp:
    def __init__(self, root, ros: HmiRosBridge):
        self.root = root
        self.ros = ros
        self.root.title("Vision AI Reagent QC - HMI")

        self._build_layout()
        self.root.after(150, self._refresh)

    def _build_layout(self):
        main = ttk.Frame(self.root, padding=8)
        main.grid(row=0, column=0, sticky="nsew")

        # Camera View
        cam_frame = ttk.LabelFrame(main, text="Camera View")
        cam_frame.grid(row=0, column=0, rowspan=3, padx=4, pady=4, sticky="n")
        self.camera_label = ttk.Label(cam_frame, text="(no image)")
        self.camera_label.pack(padx=4, pady=4)

        # Tube State Panel
        tube_frame = ttk.LabelFrame(main, text="Tube State Panel")
        tube_frame.grid(row=0, column=1, padx=4, pady=4, sticky="new")
        self.tube_widgets = []
        for i in range(3):
            row = ttk.Frame(tube_frame)
            row.pack(fill="x", padx=4, pady=2)
            ttk.Label(row, text=f"Tube {i}:", width=8).pack(side="left")
            color_box = tk.Label(row, text="    ", bg="#888888", width=4)
            color_box.pack(side="left", padx=4)
            state_text = ttk.Label(row, text="UNKNOWN", width=16)
            state_text.pack(side="left")
            height_text = ttk.Label(row, text="-- mm", width=10)
            height_text.pack(side="left")
            self.tube_widgets.append((color_box, state_text, height_text))

        # Robot Status Panel
        robot_frame = ttk.LabelFrame(main, text="Robot Status Panel")
        robot_frame.grid(row=1, column=1, padx=4, pady=4, sticky="new")
        self.robot_status_label = ttk.Label(robot_frame, text="status: --")
        self.robot_status_label.pack(anchor="w", padx=4, pady=1)
        self.robot_task_label = ttk.Label(robot_frame, text="task: --")
        self.robot_task_label.pack(anchor="w", padx=4, pady=1)
        self.robot_detail_label = ttk.Label(robot_frame, text="detail: --")
        self.robot_detail_label.pack(anchor="w", padx=4, pady=1)

        # Safety Panel
        safety_frame = ttk.LabelFrame(main, text="Safety Panel")
        safety_frame.grid(row=2, column=1, padx=4, pady=4, sticky="new")
        self.camera_status_label = ttk.Label(safety_frame, text="Camera: --")
        self.camera_status_label.pack(anchor="w", padx=4, pady=1)
        self.hand_status_label = ttk.Label(safety_frame, text="Hand in work area: --")
        self.hand_status_label.pack(anchor="w", padx=4, pady=1)

        # Control Panel
        control_frame = ttk.LabelFrame(main, text="Control Panel")
        control_frame.grid(row=3, column=0, columnspan=2, padx=4, pady=4, sticky="ew")
        ttk.Button(control_frame, text="Start", command=self.on_start).pack(side="left", padx=4, pady=4)
        ttk.Button(control_frame, text="Stop", command=self.on_stop).pack(side="left", padx=4, pady=4)
        ttk.Button(control_frame, text="Recheck", command=self.on_recheck).pack(side="left", padx=4, pady=4)
        ttk.Button(
            control_frame, text="EMERGENCY STOP", command=self.on_emergency_stop
        ).pack(side="left", padx=12, pady=4)

        # Log Panel
        log_frame = ttk.LabelFrame(main, text="Log Panel")
        log_frame.grid(row=4, column=0, columnspan=2, padx=4, pady=4, sticky="nsew")
        self.log_text = tk.Text(log_frame, height=8, width=70, state="disabled")
        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)

    def log(self, message):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{ts}] {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    # ---- control panel callbacks ----

    def on_start(self):
        future = self.ros.call_start_task()
        if future is None:
            self.log("Start: no tube_state received yet")
            return
        self.log("Start requested")
        future.add_done_callback(lambda f: self._log_service_result("start_task", f))

    def on_stop(self):
        future = self.ros.call_stop_task()
        self.log("Stop requested")
        future.add_done_callback(lambda f: self._log_service_result("stop_task", f))

    def on_recheck(self):
        future = self.ros.call_request_recheck()
        self.log("Recheck requested")
        future.add_done_callback(lambda f: self._log_service_result("request_recheck", f))

    def on_emergency_stop(self):
        future = self.ros.call_stop_task()
        self.log("EMERGENCY STOP pressed")
        future.add_done_callback(lambda f: self._log_service_result("stop_task", f))

    def _log_service_result(self, name, future):
        try:
            result = future.result()
            self.log(f"{name} -> success={result.success} message={result.message}")
        except Exception as e:
            self.log(f"{name} -> error: {e}")

    # ---- periodic refresh from cached ROS state ----

    def _refresh(self):
        self._refresh_camera()
        self._refresh_tube_state()
        self._refresh_robot_status()
        self._refresh_safety()
        self.root.after(150, self._refresh)

    def _refresh_camera(self):
        frame = self.ros.latest_frame
        if frame is None:
            return
        img = PILImage.fromarray(frame[:, :, ::-1])  # bgr -> rgb
        img.thumbnail((480, 360))
        photo = ImageTk.PhotoImage(img)
        self.camera_label.configure(image=photo, text="")
        self.camera_label.image = photo  # keep a reference

    def _refresh_tube_state(self):
        state_msg = self.ros.latest_tube_state
        height_msg = self.ros.latest_tube_height
        heights = {}
        if height_msg is not None:
            heights = dict(zip(height_msg.tube_index, height_msg.liquid_height))

        if state_msg is None:
            return
        for idx, state in zip(state_msg.tube_index, state_msg.state):
            if idx >= len(self.tube_widgets):
                continue
            color_box, state_text, height_text = self.tube_widgets[idx]
            color_box.configure(bg=STATE_COLORS.get(state, "#888888"))
            state_text.configure(text=STATE_LABELS.get(state, str(state)))
            if idx in heights:
                height_text.configure(text=f"{heights[idx]:.1f} mm")

    def _refresh_robot_status(self):
        status_msg = self.ros.latest_robot_status
        if status_msg is None:
            return
        self.robot_status_label.configure(text=f"status: {status_msg.status}")
        self.robot_task_label.configure(text=f"task: {status_msg.current_task or '--'}")
        self.robot_detail_label.configure(text=f"detail: {status_msg.detail or '--'}")

    def _refresh_safety(self):
        self.camera_status_label.configure(
            text=f"Camera: {'OK' if self.ros.camera_ok else 'DISCONNECTED'}"
        )
        self.hand_status_label.configure(
            text=f"Hand in work area: {'YES - motion withheld' if self.ros.hand_detected else 'clear'}"
        )


def main(args=None):
    rclpy.init(args=args)
    ros_node = HmiRosBridge()

    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    HmiApp(root, ros_node)
    try:
        root.mainloop()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
