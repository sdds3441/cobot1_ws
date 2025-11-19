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

def perform_task(publisher=None, task_name="Squeegee Wipe", progress_offset=0.0, progress_weight=100.0, cleanliness_offset=0.0, cleanliness_weight=100.0):
    """로봇이 수행할 작업 (스퀴지 와이퍼 동작) 및 상태 발행"""
    print(f"Performing task: {task_name}...")
    from DSR_ROBOT2 import posx, movej, movel

    # --- 1. 'ㄹ' 모양 파라미터 정의 (sky_cleaner.py에서 가져옴) ---
    JReady = [0, 0, 90, 0, 90, 0]
    Y_CONST = -590
    ORIENTATION = [90, -90, 0]  # Rx, Ry, Rz 값
    z_levels = [500, 450, 400, 300]
    X_MAX = 500  
    X_MIN = -210
    
    # --- 2. 파라미터를 이용해 좌표 리스트 생성 ---
    print("Generating 'tapered-Z' path points...")
    path_points = []
    for i, z in enumerate(z_levels):
        if i % 2 == 0:
            # 짝수 레벨 (0, 2): X_MAX -> X_MIN (우 -> 좌)
            path_points.append(posx([X_MAX, Y_CONST, z, *ORIENTATION]))
            path_points.append(posx([X_MIN, Y_CONST, z, *ORIENTATION]))
        else:
            # 홀수 레벨 (1, 3): X_MIN -> X_MAX (좌 -> 우)
            path_points.append(posx([X_MIN, Y_CONST, z, *ORIENTATION]))
            path_points.append(posx([X_MAX, Y_CONST, z, *ORIENTATION]))

    # --- 3. 로봇 이동 실행 및 상태 발행 ---
    print("--- Starting 'ㄹ' shape movement ---")
    
    num_points = len(path_points)
    for i, point in enumerate(path_points):
        print(f"Moving to point {i+1}/{num_points}...")
        movel(point, vel=VELOCITY, acc=ACC)

        if publisher:
            # 이 태스크 내부의 진행률 (0-100%)
            internal_progress = (i + 1) / num_points * 100.0
            
            # 가중치와 오프셋을 적용하여 전체 진행률 및 청결도 계산
            total_progress = progress_offset + (internal_progress / 100.0) * progress_weight
            cleanliness = cleanliness_offset + (internal_progress / 100.0) * cleanliness_weight
            
            msg = Cleanliness()
            msg.task_name = task_name
            msg.progress_percentage = total_progress
            msg.cleanliness_level = cleanliness
            publisher.publish(msg)
            print(f"  - Status Published: Task={msg.task_name}, Progress={msg.progress_percentage:.1f}%, Cleanliness={msg.cleanliness_level:.1f}%")

    # 마지막 지점 추가 (sky_cleaner.py 참고)
    ready=posx([500,-590,500,90,-90,0])
    movel(ready,vel=VELOCITY,acc=ACC)
    print(f"--- {task_name} complete ---")

def main(args=None):
    """메인 함수: 단독 실행 테스트용"""
    rclpy.init(args=args)
    node = rclpy.create_node("wriper_standalone_test")
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
