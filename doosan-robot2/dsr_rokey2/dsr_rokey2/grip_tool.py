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
    from DSR_ROBOT2 import posx,movej,movel,DR_TOOL,wait # 필요한 기능만 임포트 
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