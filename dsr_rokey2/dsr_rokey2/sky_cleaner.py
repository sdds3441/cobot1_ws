import rclpy
import DR_init

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight" #본인 TCP 이름 설정
ROBOT_TOOL = "GripperDA_v1"  #본인 그리퍼 이름 설정

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 200
ACC = 200

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
    from DSR_ROBOT2 import posx, movej, movel,task_compliance_ctrl,set_ref_coord,set_desired_force,wait,DR_FC_MOD_REL,release_compliance_ctrl,release_force  # 필요한 기능만 임포트

    # --- 1. 'ㄹ' 모양 파라미터 정의 ---
    # Y축, 손목 방향은 고정
    Y_CONST = -440
    ORIENTATION = [90, -90, 0]  # Rx, Ry, Rz 값

    # Z 레벨 (위, 중간, 아래)
    z_levels = [450, 350, 250]

    # X 너비 (아래로 갈수록 좁아짐)
    X_MAX = 520  
    X_MIN=0
    # X_MIN = -180
    
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
    #task_compliance_ctrl([200.00, 200.00, 10.00, 10.00, 10.00, 10.00])
    #wait(0.5) 
    # for 반복문: path_points 리스트의 모든 점을 순서대로 방문
    for i, point in enumerate(path_points):
        # if i == 0:
        #     movel(([520,Y_CONST-20,450,90,-90,0]), vel=VELOCITY, acc=ACC)
        print(f"Moving to point {i+1}...")
        movel(point, vel=VELOCITY, acc=ACC)

    #release_compliance_ctrl()

    ready=posx([520,Y_CONST+40,450,90,-90,0])
    movel(ready,vel=VELOCITY,acc=ACC)
    print("--- 'ㄹ' 모양 그리기 완료 ---")

def cloth_task():
    from DSR_ROBOT2 import posx,movejx,movel,DR_FC_MOD_REL, wait, set_ref_coord,task_compliance_ctrl,set_desired_force  # 필요한 기능만 임포트
    from DSR_ROBOT2 import release_compliance_ctrl, release_force
    # 초기 위치 및 목표 위치 설정
    home = [0, 0, 90, 0, 90, 0]
    y_val= -500
    #ready_pos = posx([300, y_val, 500, 90, -90, 90])
    ready_pos = posx([300, y_val, 500, 90, -90, 0])
    attach=posx([0,-10,0,0,0,0])
    down = posx([0,0,-300,0,0,0])

    movel(ready_pos, vel=VELOCITY, acc=ACC)
    
    for i in range(1,10):   #실전에선 6    

        task_compliance_ctrl([200.00, 100.00, 10.00, 10.00, 10.00, 10.00])
        wait(0.2) 
        movel(attach,vel=VELOCITY,acc=ACC,mod=DR_FC_MOD_REL)
        movel(down, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
        #movel(detach, vel=VELOCITY, acc=ACC)
        #detach[0]=detach[0]-120
        release_compliance_ctrl()

        if i<9: #실전에선 5
            #next_pos=posx([ready_pos[0]-100*i,y_val+10,500,90,-90,90])
            next_pos=posx([ready_pos[0]-70*i,y_val+10,500, 90, -90, 0])
            movel(next_pos, vel=VELOCITY, acc=ACC)

    movel(([300, y_val+20, 500, 90, -90, 0]), vel=VELOCITY, acc=ACC)


def pickup_roller():
    from DSR_ROBOT2 import posx,movej,movel,DR_TOOL,wait # 필요한 기능만 임포트 
    from DSR_ROBOT2 import set_digital_output, wait,DR_FC_MOD_REL
    roller= posx([546.98, 122.98, 363.38, 17.81, -178.75, -70.88])
    down=posx([0,0,-30,0,0,0,])
    pickup=posx([0, 0,150, 0, 0, 0])
    safe_spot=posx([0,-100,100,0,0,0])
    movel(roller, vel=VELOCITY,acc=ACC)
    set_digital_output(1,0)
    set_digital_output(2,1)
    wait(1)
    movel(down, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    set_digital_output(1,1)
    set_digital_output(2,1)
    wait(1)
    movel(pickup, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    movel(safe_spot, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    print('roller pick up complete')
    tool="roller"


def return_roller():
    from DSR_ROBOT2 import posx,movej,movel,DR_TOOL,wait # 필요한 기능만 임포트 
    from DSR_ROBOT2 import set_digital_output, wait,DR_FC_MOD_REL
    roller= posx([546.98, 122.98, 363.38, 17.81, -178.75, -70.88])
    down=posx([0,0,-30,0,0,0])
    pickup=posx([0, 0, 142.34, 0, 0, 0])
    safe_spot=posx([566.98,22.98,400.38,17.81, -178.75, -70.88])
    movel(safe_spot, vel=VELOCITY,acc=ACC)
    wait(2)
    movel(roller, vel=VELOCITY,acc=ACC)
    wait(1)
    print('safe position reached')
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
    cloth = posx([461, -73.29, 325.61, 83.38, -179.74, -4.56])
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
    cloth = posx([461, -73.29, 325.61, 83.38, -179.74, -4.56])
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
    #movel(up,vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    #movel(safe,vel=VELOCITY,acc=ACC,mod=DR_FC_MOD_REL)
    pickup_roller()
    roller_task()
    return_roller()
    pickup_cloth()
    cloth_task()
    return_cloth()

    movej(home,vel=VELOCITY,acc=ACC)

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