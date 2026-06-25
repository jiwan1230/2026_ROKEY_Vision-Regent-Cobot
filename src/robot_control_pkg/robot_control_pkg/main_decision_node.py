"""Main PC decision logic.

Watches /vision/tube_state (plus the /vision/camera_status and
/vision/hand_detected safety signals) and, whenever a new actionable state
snapshot arrives and the robot is idle, calls /robot/start_task with that
snapshot. robot_task_manager_node only ever acts on the highest-priority
tier present (State2 > State0 > State1), so this node's reactive loop is
what drives the multi-cycle scenarios (A/B/D) in the spec doc to
completion: each recheck triggers a fresh /vision/tube_state, which is
re-evaluated here and may trigger the next tier's task.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from interfaces.msg import TubeState
from interfaces.srv import StartTask, StopTask


class MainDecisionNode(Node):
    def __init__(self):
        super().__init__("main_decision_node")

        self.declare_parameter("num_tubes", 3)
        self.num_tubes = int(self.get_parameter("num_tubes").value)

        self.busy = False
        self.camera_ok = True
        self.hand_detected = False
        self.tray_transferred = False
        # 손 감지 시 자동 비상정지 반응의 런타임 on/off. 끄더라도 hand_detected
        # 구독/표시는 계속 갱신되고, stop_task 호출과 task 보류만 건너뜀.
        self.hand_safety_enabled = True
        # HMI의 Start 버튼으로만 켜지는 게이트. 꺼져있으면(기본값) vision이 뭘 보내도
        # 자동 dispatch를 안 함 - HMI가 직접 start_task를 호출하는 것과 이 자동 루프가
        # 동시에 같은 서비스를 부르는 충돌을 막기 위함.
        self.system_running = False

        self.start_task_client = self.create_client(StartTask, "/robot/start_task")
        self.stop_task_client = self.create_client(StopTask, "/robot/stop_task")

        self.create_subscription(TubeState, "/vision/tube_state", self.on_tube_state, 10)
        self.create_subscription(Bool, "/vision/camera_status", self.on_camera_status, 10)
        self.create_subscription(Bool, "/vision/hand_detected", self.on_hand_detected, 10)

        # HMI 등 다른 클라이언트가 토글하고 현재 상태를 구독할 수 있게 노출
        self.hand_safety_enabled_pub = self.create_publisher(
            Bool, "/robot/hand_safety_enabled", 10
        )
        self.create_service(
            SetBool, "/robot/set_hand_safety_enabled", self.handle_set_hand_safety_enabled
        )
        self.publish_hand_safety_enabled()

        # HMI Start/Stop 버튼이 토글하고, 다른 클라이언트가 현재 상태를 구독할 수 있게 노출
        self.system_running_pub = self.create_publisher(Bool, "/robot/system_running", 10)
        self.create_service(
            SetBool, "/robot/set_system_running", self.handle_set_system_running
        )
        self.publish_system_running()

        self.get_logger().info("main_decision_node ready")

    def publish_hand_safety_enabled(self):
        self.hand_safety_enabled_pub.publish(Bool(data=self.hand_safety_enabled))

    def handle_set_hand_safety_enabled(self, request, response):
        self.hand_safety_enabled = request.data
        self.publish_hand_safety_enabled()
        response.success = True
        response.message = (
            f"hand-detected auto-stop {'enabled' if self.hand_safety_enabled else 'disabled'}"
        )
        self.get_logger().warn(response.message)
        return response

    def publish_system_running(self):
        self.system_running_pub.publish(Bool(data=self.system_running))

    def handle_set_system_running(self, request, response):
        self.system_running = request.data
        self.publish_system_running()
        if self.system_running:
            # Start는 자동 루프를 켜는 것과 동시에, 멈춰서 대기 중인 작업이 있다면
            # 그것도 같이 재개시킨다 (stop_event.clear()).
            self.stop_task_client.call_async(StopTask.Request(stop=False))
        response.success = True
        response.message = f"system_running set to {self.system_running}"
        self.get_logger().warn(response.message)
        return response

    def on_camera_status(self, msg: Bool):
        self.camera_ok = msg.data

    def on_hand_detected(self, msg: Bool):
        was_detected = self.hand_detected
        self.hand_detected = msg.data
        if not self.hand_safety_enabled:
            return
        if msg.data and not was_detected and self.busy:
            self.get_logger().warn("Hand detected in work area - requesting emergency stop")
            self.stop_task_client.call_async(StopTask.Request(stop=True))
        elif not msg.data and was_detected and self.busy:
            self.get_logger().warn("Hand cleared from work area - resuming paused task")
            self.stop_task_client.call_async(StopTask.Request(stop=False))

    def on_tube_state(self, msg: TubeState):
        if self.busy:
            return
        if not self.system_running:
            return
        if not self.camera_ok:
            self.get_logger().warn("Camera not OK - robot motion withheld")
            return
        if self.hand_detected and self.hand_safety_enabled:
            self.get_logger().warn("Hand detected in work area - robot motion withheld")
            return
        if TubeState.STATE_UNKNOWN in msg.state:
            self.get_logger().warn(f"Unknown tube state present {list(msg.state)} - withholding task")
            return
        if len(msg.tube_index) < self.num_tubes:
            self.get_logger().warn("Incomplete tube state array - withholding task")
            return

        all_normal = all(s == TubeState.STATE_NORMAL for s in msg.state)
        if all_normal and self.tray_transferred:
            return  # already transferred; nothing new to do until a tube deviates again
        if not all_normal:
            self.tray_transferred = False

        if not self.start_task_client.service_is_ready():
            self.get_logger().warn("/robot/start_task not available yet")
            return

        request = StartTask.Request(tube_index=list(msg.tube_index), state=list(msg.state))
        self.busy = True
        self.get_logger().info(f"Dispatching start_task for state={list(msg.state)}")
        future = self.start_task_client.call_async(request)
        future.add_done_callback(lambda f: self.on_start_task_done(f, all_normal))

    def on_start_task_done(self, future, was_all_normal):
        self.busy = False
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"start_task call failed: {e}")
            return

        self.get_logger().info(f"start_task result: success={result.success} message={result.message}")
        if was_all_normal and result.success:
            self.tray_transferred = True


def main(args=None):
    rclpy.init(args=args)
    node = MainDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
