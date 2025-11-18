import rclpy
import DR_init
from dsr_interfaces.msg import Cleanliness

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

def perform_task(publisher=None, task_name="Force Control Wipe", progress_offset=0.0, progress_weight=100.0, cleanliness_offset=0.0, cleanliness_weight=100.0):
    """로봇이 수행할 작업 (힘 제어 기반 반복 동작) 및 상태 발행"""
    print(f"Performing task: {task_name}...")
    from DSR_ROBOT2 import posx,movej,movel,DR_FC_MOD_REL, wait, set_ref_coord,task_compliance_ctrl,set_desired_force  # 필요한 기능만 임포트
    from DSR_ROBOT2 import release_compliance_ctrl, release_force
    
    # 초기 위치 및 목표 위치 설정 (sky_cleaner.py에서 가져옴)
    home = [0, 0, 90, 0, 90, 0]
    ready_pos = posx([500, -580, 500, 90, -90, 90])
    attach=posx([0,-40,0,0,0,0])
    detach=posx([500,-580,300,90,-90,90])
    down = posx([0,0,-200,0,0,0])
    next_pos=posx([-80,0,200,0,0,0]) # 'next'는 예약어일 수 있으므로 'next_pos'로 변경
    
    # 반복 동작 수행
    movel(ready_pos, vel=VELOCITY, acc=ACC)
    
    num_repetitions = 3
    for i in range(num_repetitions):       
        print(f"--- Repetition {i+1}/{num_repetitions} ---")

        set_ref_coord(1)
        task_compliance_ctrl([3000.00, 3000.00, 10.00, 200.00, 200.00, 200.00])
        wait(0.5) 
        set_desired_force(fd=[0, 0, 5, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], time=0, mod=DR_FC_MOD_REL)
        set_ref_coord(0)
        wait(0.5)
        movel(attach,vel=VELOCITY,acc=ACC,mod=DR_FC_MOD_REL)
        movel(down, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
        movel(detach, vel=VELOCITY, acc=ACC)
        detach[0]=detach[0]-80
        wait(0.5)
        set_ref_coord(1)
        release_force(time=0.0)
        release_compliance_ctrl()
        set_ref_coord(0)
        movel(next_pos, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)

        if publisher:
            # 이 태스크 내부의 진행률 (0-100%)
            internal_progress = (i + 1) / num_repetitions * 100.0
            
            # 가중치와 오프셋을 적용하여 전체 진행률 및 청결도 계산
            total_progress = progress_offset + (internal_progress / 100.0) * progress_weight
            cleanliness = cleanliness_offset + (internal_progress / 100.0) * cleanliness_weight
            
            msg = Cleanliness()
            msg.task_name = task_name
            msg.progress_percentage = total_progress
            msg.cleanliness_level = cleanliness
            publisher.publish(msg)
            print(f"  - Status Published: Task={msg.task_name}, Progress={msg.progress_percentage:.1f}%, Cleanliness={msg.cleanliness_level:.1f}%")
    
    movel(ready_pos, vel=VELOCITY, acc=ACC)
    print(f"--- {task_name} complete ---")

def main(args=None):
    """메인 함수: 단독 실행 테스트용"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_vertical_standalone_test")
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        # 단독 테스트 시에는 퍼블리셔 없이 실행
        perform_task()
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
