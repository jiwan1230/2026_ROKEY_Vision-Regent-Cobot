"""Mock Doosan M0609 arm motion node.

Hosts /robot/move_to_pose. Each named pose is an [x, y, z, rx, ry, rz]
posx-style coordinate loaded from config/robot_params.yaml (spec doc
section 13's coordinate map). The actual motion is simulated with a sleep
proportional to move_duration_sec, standing in for movej/movel + mwait().

To connect to the real robot, replace _simulate_move() with calls into
doosan_robot2 (DSR_ROBOT2.movel / mwait), keeping pose lookup and the
service interface unchanged.
"""
import time
import sys
import rclpy
import math
import copy
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from dsr_msgs2.srv import MoveStop
from interfaces.srv import MoveToPose, AddTcp, SetTcp
from robot_control_pkg.poses import all_pose_names

if not rclpy.ok():
    rclpy.init(args=sys.argv)

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

dsr_node = rclpy.create_node('dsr_lib_node_move', namespace=ROBOT_ID)
# movel()이 내부적으로 dsr_node를 spin_until_future_complete로 블락하는 동안에도
# 정지 요청이 끼어들 수 있어야 해서, MoveStop 클라이언트는 별도 노드에 둔다 - 같은
# 노드를 같이 쓰면 movel()이 막혀있는 동안 이 요청도 같이 막혀버림.
dsr_stop_node = rclpy.create_node('dsr_lib_node_stop', namespace=ROBOT_ID)

import DR_init
DR_init.__dsr__node = dsr_node
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
# 260625 준형님 코드로 부분 수정
# from DSR_ROBOT2 import movel, move_periodic
from DSR_ROBOT2 import movel, move_periodic, get_current_posx, get_external_torque, DR_BASE, DR_HOLD, wait
# END

#20260625 JH, 가상TCP 적용을 위한 변환 추가
def euler_zyz_to_matrix(rx, ry, rz):
    r1, r2, r3 = math.radians(rx), math.radians(ry), math.radians(rz)
    
    cz1, sz1 = math.cos(r1), math.sin(r1)
    cy, sy   = math.cos(r2), math.sin(r2)
    cz2, sz2 = math.cos(r3), math.sin(r3)

    Rz1 = np.array([[cz1, -sz1, 0], [sz1, cz1, 0], [0, 0, 1]])
    Ry  = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz2 = np.array([[cz2, -sz2, 0], [sz2, cz2, 0], [0, 0, 1]])

    return Rz1 @ Ry @ Rz2

def matrix_to_euler_zyz(R):
    sy = math.sqrt(R[0, 2]**2 + R[1, 2]**2)
    singular = sy < 1e-6
    
    if not singular:
        rx = math.atan2(R[1, 2], R[0, 2])
        ry = math.atan2(sy, R[2, 2])
        rz = math.atan2(R[2, 1], -R[2, 0])
    else:
        rx = math.atan2(-R[1, 0], R[1, 1])
        ry = math.atan2(sy, R[2, 2])
        rz = 0

    return [math.degrees(rx), math.degrees(ry), math.degrees(rz)]

def apply_virtual_tcp(target_pose, tcp_offset):
    x, y, z, rx, ry, rz = target_pose
    
    T_target = np.eye(4)
    T_target[0:3, 0:3] = euler_zyz_to_matrix(rx, ry, rz)
    T_target[0:3, 3] = [x, y, z]

    T_tcp = np.eye(4)
    T_tcp[0:3, 0:3] = euler_zyz_to_matrix(tcp_offset[3], tcp_offset[4], tcp_offset[5])
    T_tcp[0:3, 3] = tcp_offset[0:3]

    T_flange = T_target @ np.linalg.inv(T_tcp)

    new_xyz = T_flange[0:3, 3].tolist()
    new_rx_ry_rz = matrix_to_euler_zyz(T_flange[0:3, 0:3])
    
    return new_xyz + new_rx_ry_rz

def get_forward_tcp(flange_pose, tcp_offset):
    x, y, z, rx, ry, rz = flange_pose
    
    T_flange = np.eye(4)
    T_flange[0:3, 0:3] = euler_zyz_to_matrix(rx, ry, rz)
    T_flange[0:3, 3] = [x, y, z]
    
    T_tcp = np.eye(4)
    T_tcp[0:3, 0:3] = euler_zyz_to_matrix(tcp_offset[3], tcp_offset[4], tcp_offset[5])
    T_tcp[0:3, 3] = tcp_offset[0:3]

    # Flange 행렬에 TCP 행렬을 곱해서 공간상 위치 도출
    T_target = T_flange @ T_tcp
    
    new_xyz = T_target[0:3, 3].tolist()
    new_rx_ry_rz = matrix_to_euler_zyz(T_target[0:3, 0:3])
    
    return new_xyz + new_rx_ry_rz
#end

class DoosanRobotControlNode(Node):
    def __init__(self):
        super().__init__("doosan_robot_control_node")
        
        #20260625 JH, 기본 TCP 설정
        self.tcps = {}
        # self.current_tcp_name = "default_tcp"
        # self.current_tcp_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_tcp_name = "gripper_tcp"
        self.current_tcp_offset = [0.0, 0.0, 200.0, 0.0, 0.0, 0.0]
        self.tcp_rotate_offset = [0.0, 25.0, 0.0, 0.0, 0.0, 0.0]

        # 외력 감지 상태 (히스테리시스용)
        self._force_detected = False

        #나중에 HMI에서 받아오게 바꿔야 함
        self.declare_parameter("m_velocity", 60.0)
        self.declare_parameter("m_acceleration", 60.0)
        self.declare_parameter("d_velocity", 30.0)
        self.declare_parameter("d_acceleration", 30.0)
        self.declare_parameter("move_duration_sec", 0.4)
        # 외력 감지 임계값 (Nm) - 각 관절 외력 토크 중 최대값이 이 값을 초과하면 감지
        self.declare_parameter("force_threshold", 20.0)

        self.move_duration_sec = float(self.get_parameter("move_duration_sec").value)

        self.poses = {}
        for name in all_pose_names(num_tubes=3):
            param_name = f"poses.{name}"
            self.declare_parameter(param_name, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.poses[name] = list(self.get_parameter(param_name).value)

        self.current_pose_name = "home_pose"

        self.move_stop_client = dsr_stop_node.create_client(MoveStop, "motion/move_stop")

        self.force_detected_pub = self.create_publisher(Bool, "/robot/force_detected", 10)
        # 200ms 주기로 외력 모니터링
        self.create_timer(0.2, self._check_external_force)

        #20260625 JH, AddTCP, SetTCP 서비스 추가
        self.create_service(AddTcp, "/robot/add_tcp", self.handle_add_tcp)
        self.create_service(SetTcp, "/robot/set_tcp", self.handle_set_tcp)

        self.create_service(MoveToPose, "/robot/move_to_pose", self.handle_move_to_pose)
        # move_to_pose와 다른 callback group을 써야 함 - 기본(상호배제) 그룹을 같이 쓰면
        # movel()이 진행 중인 동안 이 서비스 콜백도 같은 그룹에서 큐에 걸려 대기하느라
        # 정지 요청이 모션이 끝날 때까지 전달이 안 됨.
        self.create_service(
            Trigger, "/robot/hard_stop", self.handle_hard_stop, callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info(f"doosan_robot_control_node ready ({len(self.poses)} poses loaded)")

    #20260625 JH, add_tcp 함수추가 : 이미 있는 내용인지 확인 후 추가
    def handle_add_tcp(self, request, response):
        new_name = request.tcp_name
        new_offset = list(request.tcp_offset)

        if new_name in self.tcps:
            response.success = False
            response.message = f"이미 존재하는 TCP 이름입니다: {new_name}"
            self.get_logger().warn(response.message)
        else:
            self.tcps[new_name] = new_offset
            response.success = True
            response.message = f"새로운 TCP가 등록되었습니다: {new_name} -> {new_offset}"
            self.get_logger().info(response.message)
            
        return response
    
    #20260625 JH, set_tcp 함수추가
    def handle_set_tcp(self, request, response):
        new_name = request.tcp_name
        new_offset = list(request.tcp_offset) 
        
        self.current_tcp_name = new_name
        self.current_tcp_offset = new_offset
        
        response.success = True
        response.message = f"가상 TCP 변경 완료: {new_name} -> {new_offset}"
        self.get_logger().info(response.message)
        return response

    #20260624 준형, move_type에 따라 이동방식 다르게 적용
    #20260625 준형, rotate 로직 변경
    #20260625 JH, TCP 적용 변환 추가
    def move(self, pose_name, target, move_type):
        self.get_logger().info(f"MOVE -> {pose_name} (현재 툴: {self.current_tcp_name})")
        real_target = apply_virtual_tcp(target, self.current_tcp_offset)

        if move_type == 'move':
            movel(real_target, vel=self.get_parameter("m_velocity").value, acc=self.get_parameter("m_acceleration").value)
        elif move_type == 'down':
            movel(real_target, vel=self.get_parameter("d_velocity").value, acc=self.get_parameter("d_acceleration").value)
        elif move_type == 'down_tray':
            movel(real_target, vel=self.get_parameter("d_velocity").value, acc=self.get_parameter("d_acceleration").value)
        elif move_type == 'rotate':
            current_flange = get_current_posx(DR_BASE)[0]
            tcp_rotate = [x + y for x, y in zip(self.current_tcp_offset, self.tcp_rotate_offset)]

            edge_pose = get_forward_tcp(current_flange, tcp_rotate)

            edge_pose[2] -= 1.33
            edge_pose[3] = 90.0   # A (Rx)
            edge_pose[4] -= 2.0
            edge_pose[5] = -90.0  # C (Rz)

            real_rotate_target = apply_virtual_tcp(edge_pose, tcp_rotate)

            self.get_logger().info(f"Flange 목표 좌표: {real_rotate_target}")

            movel(real_rotate_target, vel=[self.get_parameter("d_velocity").value, 5], acc=[self.get_parameter("d_acceleration").value, 5])
            move_periodic([0, 0, 0, 0, 0, 5], period=0.5, repeat=3)
            wait(0.5)
        time.sleep(self.move_duration_sec)
    #end
    
    #20260624 준형, pose 실시간 업데이트 적용
    #20260625 준형, ratio 더 이상 사용하지 않으므로 해당 내용 삭제
    def handle_move_to_pose(self, request, response):
        pose_name = request.pose_name
        param_name = f"poses.{pose_name}"
        if not self.has_parameter(param_name):
            response.success = False
            response.message = f"Unknown pose_name: {pose_name}"
            self.get_logger().error(response.message)
            return response

        target = self.get_parameter(param_name).value

        self.get_logger().info(f'{target} to move')
        self.move(pose_name, target, request.move_type)
        self.current_pose_name = pose_name

        response.success = True
        response.message = f"Moved to {pose_name}"
        return response

    # 지금 진행 중인 movel()을 하드웨어 레벨에서 즉시 정지시킨다 (DR_HOLD: 다시
    # movel()을 보내면 그대로 이어갈 수 있는 정지 모드 - STO/QSTOP처럼 안전 정지
    # 상태로 빠지지 않음). 응답을 기다릴 필요 없는 fire-and-forget이라 비동기로만 호출.
    def handle_hard_stop(self, request, response):
        if not self.move_stop_client.service_is_ready():
            response.success = False
            response.message = "motion/move_stop service not available"
            self.get_logger().error(response.message)
            return response

        future = self.move_stop_client.call_async(MoveStop.Request(stop_mode=DR_HOLD))
        future.add_done_callback(self._log_move_stop_result)
        response.success = True
        response.message = "Hard stop requested"
        self.get_logger().warn(response.message)
        return response

    def _log_move_stop_result(self, future):
        try:
            result = future.result()
            self.get_logger().warn(f"motion/move_stop -> success={result.success}")
        except Exception as e:
            self.get_logger().error(f"motion/move_stop call failed: {e}")

    def _check_external_force(self):
        try:
            torques = get_external_torque()
            threshold = float(self.get_parameter("force_threshold").value)
            max_torque = max(abs(t) for t in torques)
            exceeded = max_torque > threshold

            if exceeded and not self._force_detected:
                self._force_detected = True
                self.get_logger().warn(
                    f"External force detected: max torque={max_torque:.1f} Nm (threshold={threshold})"
                )
                self.force_detected_pub.publish(Bool(data=True))
                # 즉시 모션 정지
                if self.move_stop_client.service_is_ready():
                    self.move_stop_client.call_async(MoveStop.Request(stop_mode=DR_HOLD))
            elif not exceeded and self._force_detected:
                self._force_detected = False
                self.get_logger().info("External force cleared")
                self.force_detected_pub.publish(Bool(data=False))
        except Exception as e:
            self.get_logger().debug(f"Force check skipped: {e}")
#end

def main():
    control_node = DoosanRobotControlNode()

    executor = MultiThreadedExecutor()
    executor.add_node(control_node)
    executor.add_node(dsr_node)
    executor.add_node(dsr_stop_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        control_node.destroy_node()
        dsr_node.destroy_node()
        dsr_stop_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
