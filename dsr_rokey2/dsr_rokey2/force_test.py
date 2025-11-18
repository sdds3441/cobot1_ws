#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import DR_init

from dsr_msgs2.srv import GetToolForce   # ✅ dsr_msgs2 로!

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight" #본인 TCP 이름 설정
ROBOT_TOOL = "GripperDA_v1"  #본인 그리퍼 이름 설정

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 500
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp  # 필요한 기능만 임포트

    # Tool과 TCP 설정
    set_tool(ROBOT_TCP)
    set_tcp(ROBOT_TOOL)

    # 설정된 설정값 출력
    # print("#"*50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#"*50)

class ToolForceClient(Node):
    def __init__(self):
        super().__init__('tool_force_client')

        # 서비스 클라이언트 생성
        self.cli = self.create_client(
            GetToolForce,
            '/dsr01/aux_control/get_tool_force'
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /dsr01/aux_control/get_tool_force ...')

        self.get_logger().info('Connected to /dsr01/aux_control/get_tool_force')

        # 0.1초마다(10Hz) 주기적으로 호출
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        req = GetToolForce.Request()

        # ref: 기준 좌표계
        # 0: BASE, 1: WORLD, 2: TOOL (깃허브 이슈 기준)
        req.ref = 2   # 예: TOOL 좌표계 기준 힘을 보고 싶으면 2

        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return

        if not res.success:
            self.get_logger().warn('GetToolForce returned success = False')
            return

        # ✅ 여기! tool_force 는 길이 6짜리 배열
        # [Fx, Fy, Fz, Tx, Ty, Tz]
        tf = res.tool_force
        if len(tf) != 6:
            self.get_logger().error(f'Unexpected tool_force length: {len(tf)}')
            return

        fx, fy, fz, tx, ty, tz = tf

        self.get_logger().info(
            f'Tool Force  F: [{fx:.3f}, {fy:.3f}, {fz:.3f}] N  '
            f'T: [{tx:.3f}, {ty:.3f}, {tz:.3f}] N·m'
        )


def perform_task():

    from DSR_ROBOT2 import set_ref_coord, task_compliance_ctrl, release_compliance_ctrl

    node = ToolForceClient()
    set_ref_coord(1)
    task_compliance_ctrl([3000.00, 3000.00, 200.00, 200.00, 200.00, 200.00])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()

        # 작업 수행
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()