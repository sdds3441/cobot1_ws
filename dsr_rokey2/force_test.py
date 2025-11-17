import rclpy
from rclpy.node import Node
import DR_init


# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight" #본인 TCP 이름 설정
ROBOT_TOOL = "GripperDA_v1"  #본인 그리퍼 이름 설정

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 30
ACC = 20

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

class ExternalTorqueClient(Node):
    
    def __init__(self):
        from dsr_msgs2.srv import GetExternalTorque
        super().__init__('external_torque_client')

        # 서비스 클라이언트 생성
        self.cli = self.create_client(
            GetExternalTorque,
            '/dsr01/aux_control/get_external_torque'
        )

        # 서비스 준비될 때까지 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기중...')

    def request_torque(self):
        from dsr_msgs2.srv import GetExternalTorque
        req = GetExternalTorque.Request()  # 요청필드 없음 → 빈 객체 생성만 하면 됨

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        return future.result()

def perform_task():
    """로봇이 수행할 작업"""
    print("Performing task...")
    from DSR_ROBOT2 import posx,movej,movel, move_periodic,DR_TOOL,wait, set_stiffnessx # 필요한 기능만 임포트 
    from DSR_ROBOT2 import set_desired_force, release_force,DR_FC_MOD_REL,set_ref_coord, task_compliance_ctrl, release_compliance_ctrl
    from dsr_msgs2.srv import GetExternalTorque
    JReady = [0, 0, 90, 0, 90, 0]
    movej(JReady, vel=VELOCITY, acc=ACC)
    node = ExternalTorqueClient()

    result = node.request_torque()

    print("External Torque (Nm):")
    print(list(result.ext_torque))
    print("Success:", result.success)

    set_ref_coord(1)
    task_compliance_ctrl([3000.00, 3000.00, 3000.00, 200.00, 200.00, 200.00])
    #set_stiffnessx([3000.00, 3000.00, 3000.00, 200.00, 200.00, 200.00])
    wait(0.5) # 안정화 대기(필수)
    set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], time=0, mod=DR_FC_MOD_REL)
    
    wait(5)
    print("External Torque (Nm):")
    print(list(result.ext_torque))
    print("Success:", result.success)
    release_force(time=0.0)
    release_compliance_ctrl()
    


   
    

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