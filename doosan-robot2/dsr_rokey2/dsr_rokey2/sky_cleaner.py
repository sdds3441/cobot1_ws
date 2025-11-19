import rclpy
import DR_init

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


def roller_task():
    """로봇이 수행할 작업 ('ㄹ' 모양 및 너비 축소 적용)"""
    print("Performing task...")
    from DSR_ROBOT2 import posx, movej, movel  # 필요한 기능만 임포트

    # 초기 위치 (준비 자세)
    JReady = [0, 0, 90, 0, 90, 0]

    # --- 1. 'ㄹ' 모양 파라미터 정의 ---
    # Y축, 손목 방향은 고정
    Y_CONST = -590
    ORIENTATION = [90, -90, 0]  # Rx, Ry, Rz 값

    # Z 레벨 (위, 중간, 아래)
    z_levels = [500, 450, 400, 300]

    # X 너비 (아래로 갈수록 좁아짐)
    X_MAX = 500  
    X_MIN = -210
    
    # --- 2. 파라미터를 이용해 좌표 리스트 생성 ---
    print("Generating 'tapered-Z' path points...")
    path_points = []
    
    # * 연산자: ORIENTATION 리스트 [90, -90, 0]을 풀어서 인자로 넣어줌
    
# z_levels 리스트를 순회하며 인덱스(i)와 높이(z)를 가져옴
    for i, z in enumerate(z_levels):
        
        # * 연산자: ORIENTATION 리스트 [90, -90, 0]을 풀어서 인자로 넣어줌
        
        if i % 2 == 0:
            # 짝수 레벨 (0, 2): X_MAX -> X_MIN (우 -> 좌)
            path_points.append(posx([X_MAX, Y_CONST, z, *ORIENTATION]))
            path_points.append(posx([X_MIN, Y_CONST, z, *ORIENTATION]))
        else:
            # 홀수 레벨 (1, 3): X_MIN -> X_MAX (좌 -> 우)
            path_points.append(posx([X_MIN, Y_CONST, z, *ORIENTATION]))
            path_points.append(posx([X_MAX, Y_CONST, z, *ORIENTATION]))
    # --- 3. 로봇 이동 실행 ---


    print("--- Starting 'ㄹ' shape movement ---")
    
    # for 반복문: path_points 리스트의 모든 점을 순서대로 방문
    for i, point in enumerate(path_points):
        print(f"Moving to point {i+1}...")
        movel(point, vel=VELOCITY, acc=ACC)
    ready=posx([500,-590,500,90,-90,0])
    movel(ready,vel=VELOCITY,acc=ACC)
    print("--- 'ㄹ' 모양 그리기 완료 ---")

def cloth_task():
    from DSR_ROBOT2 import posx,movej,movel,DR_FC_MOD_REL, wait, set_ref_coord,task_compliance_ctrl,set_desired_force  # 필요한 기능만 임포트
    from DSR_ROBOT2 import release_compliance_ctrl, release_force
    # 초기 위치 및 목표 위치 설정
    home = [0, 0, 90, 0, 90, 0]
    ready_pos = posx([500, -580, 500, 90, -90, 90])
    attach=posx([0,-40,0,0,0,0])
    detach=posx([500,-580,300,90,-90,90])
    down = posx([0,0,-200,0,0,0])
    next=posx([-80,0,200,0,0,0])

    movel(ready_pos, vel=VELOCITY, acc=ACC)
    
    for i in range(3):       

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
        movel(next, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)

    movel(ready_pos, vel=VELOCITY, acc=ACC)


def pickup_roller():
    from DSR_ROBOT2 import posx,movej,movel,DR_TOOL,wait # 필요한 기능만 임포트 
    from DSR_ROBOT2 import set_digital_output, wait,DR_FC_MOD_REL
    roller= posx([524.98, 122.98, 363.38, 17.81, -178.75, -70.88])
    down=posx([0,0,-30,0,0,0,])
    pickup=posx([0, 0, 192.34, 0, 0, 0])
    movel(roller, vel=VELOCITY,acc=ACC)
    set_digital_output(1,0)
    set_digital_output(2,1)
    wait(1)
    movel(down, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    set_digital_output(1,1)
    set_digital_output(2,1)
    wait(1)
    movel(pickup, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    print('roller pick up complete')
    tool="roller"


def return_roller():
    from DSR_ROBOT2 import posx,movej,movel,DR_TOOL,wait # 필요한 기능만 임포트 
    from DSR_ROBOT2 import set_digital_output, wait,DR_FC_MOD_REL
    roller= posx([524.98, 122.98, 563.38, 17.81, -178.75, -70.88])
    down=posx([0,0,-230,0,0,0,])
    pickup=posx([0, 0, 192.34, 0, 0, 0])

    
    movel(roller, vel=VELOCITY,acc=ACC)
    
    movel(down, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    set_digital_output(1,0)
    set_digital_output(2,1)
    wait(1)
    movel(pickup, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    print('roller return complete')
    tool="empty"

def pickup_cloth():
    from DSR_ROBOT2 import posx,movej,movel,DR_TOOL,wait # 필요한 기능만 임포트 
    from DSR_ROBOT2 import set_digital_output, wait,DR_FC_MOD_REL
    cloth = posx([610.68, -73.29, 325.61, 83.38, -179.74, -4.56])
    down=posx([0,0,-70,0,0,0,])
    pickup=posx([0, 0, 115.16, 0, 0, 0])
    
    movel(cloth, vel=VELOCITY,acc=ACC)
    set_digital_output(1,0)
    set_digital_output(2,1)
    wait(1)
    
    movel(down, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    set_digital_output(1,1)
    set_digital_output(2,0)
    wait(1)
    movel(pickup, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    print('cloth pick up complete')


def return_cloth():
    from DSR_ROBOT2 import posx,movel,DR_TOOL,wait # 필요한 기능만 임포트 
    from DSR_ROBOT2 import set_digital_output, wait,DR_FC_MOD_REL
    cloth = posx([610.68, -73.29, 325.61, 83.38, -179.74, -4.56])
    down=posx([0,0,-70,0,0,0,])
    pickup=posx([0, 0, 115.16, 0, 0, 0])

    
    movel(cloth, vel=VELOCITY,acc=ACC)

    
    movel(down, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    set_digital_output(1,0)
    set_digital_output(2,1)
    wait(1)
    movel(pickup, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    print('roller return complete')
    tool="empty"
def perform_task():
    """로봇이 수행할 작업"""
    print("Performing task...")


    from DSR_ROBOT2 import posx,movej,movel,DR_FC_MOD_REL, wait, set_ref_coord,task_compliance_ctrl,set_desired_force  # 필요한 기능만 임포트
    from DSR_ROBOT2 import release_compliance_ctrl, release_force
    # 초기 위치 및 목표 위치 설정
    home = [0, 0, 90, 0, 90, 0]
    up=posx([0,0,100,0,0,0])
    safe=posx([150,0,0,0,0,0])
    movej(home,vel=VELOCITY,acc=ACC)
    movel(up,vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    movel(safe,vel=VELOCITY,acc=ACC,mod=DR_FC_MOD_REL)
    pickup_roller()
    roller_task()
    return_roller()
    pickup_cloth()
    cloth_task()
    return_cloth()

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