import rclpy
import DR_init
from dsr_interfaces.msg import Cleanliness
# 메시지 발행 기능이 포함된 '작업' 모듈 임포트
from dsr_rokey2.wriper_msg import perform_task as roller_task
from dsr_rokey2.move_vertical_ooo import perform_task as cloth_task

# 유틸리티로 변경된 'grip_tool' 모듈에서 개별 함수 임포트
from dsr_rokey2.grip_tool import initialize_robot, pickup_roller, return_roller, pickup_cloth, return_cloth

#/task/start -> 0,1,2,3 청소 -> 제일 청소 시작 3번. 
# /task/end -> boolean형식 작업 종료 -> endwork 누르면 True
# /task/emergency_stop -> 긴급상황 정지 -> boolean 누르면 true 
VELOCITY = 500
ACC = 60
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

def main(args=None):
    """
    하나의 ROS 노드를 초기화하고, 새로운 순서에 따라 작업들을 순차적으로 실행하며 상태를 발행합니다.
    """
    rclpy.init(args=args)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    node = rclpy.create_node("master_runner_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    from DSR_ROBOT2 import posx,movej,movel,DR_FC_MOD_REL, wait,set_ref_coord,task_compliance_ctrl,set_desired_force 
    status_publisher = node.create_publisher(Cleanliness, 'cleaning_status', 10)
    
    def publish_status(task_name, progress, cleanliness):
        msg = Cleanliness()
        msg.task_name = task_name
        msg.progress_percentage = float(progress)
        msg.cleanliness_level = float(cleanliness)
        status_publisher.publish(msg)
        node.get_logger().info(f"Published: Task={msg.task_name}, Progress={msg.progress_percentage:.1f}%, Cleanliness={msg.cleanliness_level:.1f}%")

    try:
        initialize_robot()

        # --- sky_cleaner.py에 따른 새로운 작업 순서 및 진행률 할당 ---
        # 1. pickup_roller:    Progress 0-5%
        # 2. roller_task:      Progress 5-45% (weight 40), Cleanliness 0-50% (weight 50)
        # 3. return_roller:    Progress 45-50%
        # 4. pickup_cloth:     Progress 50-55%
        # 5. cloth_task:       Progress 55-95% (weight 40), Cleanliness 50-100% (weight 50)
        # 6. return_cloth:     Progress 95-100%

        # /task_ -> 3번 
        # --- 작업 시작 ---
        home = [0, 0, 90, 0, 90, 0]
        up=posx([0,0,100,0,0,0])
        safe=posx([150,0,0,0,0,0])
        movej(home,vel=VELOCITY,acc=ACC)
        movel(up,vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
        movel(safe,vel=VELOCITY,acc=ACC,mod=DR_FC_MOD_REL)
        
        publish_status("Sequence Start", 0.0, 0.0)

        # 1. 롤러 집기
        pickup_roller()
        publish_status("Pickup Roller", 5.0, 0.0)

        # 2. 롤러 작업 ('ㄹ'자 닦기)
        roller_task(
            publisher=status_publisher, 
            task_name="Roller Wipe",
            progress_offset=5.0,
            progress_weight=40.0,
            cleanliness_offset=0.0,
            cleanliness_weight=50.0
        )
        
        # 3. 롤러 반납
        return_roller()
        publish_status("Return Roller", 50.0, 50.0)

        # 4. 걸레 집기
        pickup_cloth()
        publish_status("Pickup Cloth", 55.0, 50.0)

        # 5. 걸레 작업 (힘 제어 닦기)
        cloth_task(
            publisher=status_publisher, 
            task_name="Cloth Wipe (Force)",
            progress_offset=55.0,
            progress_weight=40.0,
            cleanliness_offset=50.0,
            cleanliness_weight=50.0
        )

        # 6. 걸레 반납
        return_cloth()
        publish_status("Return Cloth & Sequence End", 100.0, 100.0)

        print("\n--- 모든 순차 작업이 성공적으로 완료되었습니다! ---")

    except KeyboardInterrupt:
        print("\n사용자에 의해 노드가 중단되었습니다. 종료합니다...")
    except Exception as e:
        print(f"예상치 못한 오류가 발생했습니다: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
