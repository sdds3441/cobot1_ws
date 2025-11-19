import rclpy
import DR_init
from rclpy.node import Node
from std_msgs.msg import Int32
from dsr_interfaces.msg import Cleanliness

# 메시지 발행 기능이 포함된 '작업' 모듈 임포트
from dsr_rokey2.wriper_msg import perform_task as roller_task
from dsr_rokey2.move_vertical_ooo import perform_task as cloth_task
from dsr_rokey2.grip_tool import initialize_robot, pickup_roller, return_roller, pickup_cloth, return_cloth
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY = 500
ACC = 60

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_runner_node', namespace=ROBOT_ID)
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        #node = rclpy.create_node("master_runner_node", namespace=ROBOT_ID)
        DR_init.__dsr__node = self
        #from DSR_ROBOT2 import posx,movej,movel,DR_FC_MOD_REL, wait,set_ref_coord,task_compliance_ctrl,set_desired_force
        
        # --- 퍼블리셔 및 구독자 생성 ---
        self.status_publisher = self.create_publisher(Cleanliness, 'cleaning_status', 10)
        self.task_subscriber = self.create_subscription(
            Int32,
            '/task/start',
            self.task_start_callback,
            10)
        
    # [추가] '/task/start' 메시지를 받았을 때 실행될 함수
    def task_start_callback(self, msg):
        self.get_logger().info(f"작업 시작 명령 수신 (ID: {msg.data}). 전체 시퀀스를 실행합니다.")
        
        # 메시지를 받았을 때만 running_sequence를 실행
        self.running_sequence()
        
        self.get_logger().info("--- 모든 시퀀스 완료. 다음 명령 대기 중 ---")
    # 상태 발행을 위한 퍼블리셔 생성
    #status_publisher = node.create_publisher(Cleanliness, 'cleaning_status', 10)
    
    # 작업 시작/완료 시 메시지를 발행하는 헬퍼 함수
    def publish_status(self, task_name, progress, cleanliness):
        msg = Cleanliness()
        msg.task_name = task_name
        msg.progress_percentage = float(progress)
        msg.cleanliness_level = float(cleanliness)
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Published: Task={msg.task_name}, Progress={msg.progress_percentage:.1f}%, Cleanliness={msg.cleanliness_level:.1f}%")

    def running_sequence(self):
            from DSR_ROBOT2 import posx,movej,movel,DR_FC_MOD_REL
            initialize_robot()
            home = [0, 0, 90, 0, 90, 0]
            up=posx([0,0,100,0,0,0])
            safe=posx([150,0,0,0,0,0])
            movej(home,vel=VELOCITY,acc=ACC)
            movel(up,vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
            movel(safe,vel=VELOCITY,acc=ACC,mod=DR_FC_MOD_REL)

            # --- 작업들을 순차적으로 실행하며 상태 발행 ---

            # 1. grip_tool.py 작업
            print("\n>>> [TASK 1] 'grip_tool.py' 작업을 시작합니다...")
            self.publish_status("Grip Task", 0.0, 0.0)
            pickup_roller()
            self.publish_status("Grip Task", 100.0, 10.0) # 임의의 청결도 값(10)
            print(">>> 'grip_tool.py' 작업을 완료했습니다.")

            # 2. wriper_msg.py 작업
            print("\n>>> [TASK 2] 'wriper_msg.py' 작업을 시작합니다...")
            # perform_wriper_task는 내부에서 발행하므로 publisher를 전달합니다.
            # 2. 롤러 작업 ('ㄹ'자 닦기)
            roller_task(
                publisher=self.status_publisher, 
                task_name="Roller Wipe",
                progress_offset=5.0, progress_weight=40.0,
                cleanliness_offset=0.0, cleanliness_weight=50.0
            )
            print(">>> 'wriper_msg.py' 작업을 완료했습니다.")
            # 3. 롤러 반납
            return_roller()
            self.publish_status("Return Roller", 50.0, 50.0)

            # 4. 걸레 집기
            pickup_cloth()
            self.publish_status("Pickup Cloth", 55.0, 50.0)

            # 5. 걸레 작업 (힘 제어 닦기)
            cloth_task(
                publisher=self.status_publisher, 
                task_name="Cloth Wipe (Force)",
                progress_offset=55.0, progress_weight=40.0,
                cleanliness_offset=50.0, cleanliness_weight=50.0
            )

            # 6. 걸레 반납
            return_cloth()
            self.publish_status("Return Cloth & Sequence End", 100.0, 100.0)

            self.get_logger().info("\n--- 모든 순차 작업이 성공적으로 완료되었습니다! ---")


def main(args=None):
    rclpy.init(args=args)
    node=MasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n사용자에 의해 노드가 중단되었습니다...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
