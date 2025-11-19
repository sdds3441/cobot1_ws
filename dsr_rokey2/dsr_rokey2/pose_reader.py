import rclpy
import DR_init
from std_msgs.msg import String
from rclpy.executors import SingleThreadedExecutor
import threading

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"
ROBOT_TOOL = "GripperDA_v1"

stoped=False
state="ready"

VELOCITY = 500
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 토픽으로 받은 최신 명령 저장용
latest_cmd = {"data": None}


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp

    set_tool(ROBOT_TCP)
    set_tcp(ROBOT_TOOL)

    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)


def check_emergency():
    """
    토픽에서 '50' 들어오면 홈으로 이동시키고 True 리턴.
    긴 동작(perform_task / roller_task / cloth_task) 중간에 수시로 호출.
    """
    from DSR_ROBOT2 import movej

    print(f"[EMERGENCY CHECK] latest_cmd = {latest_cmd['data']}")

    if latest_cmd["data"] == "50":
        print("[EMERGENCY] 50 신호 감지 → 홈 자세로 이동")
        movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
        latest_cmd["data"] = None
        stoped=True
        return True

    return False


def roller_task():
    global state
    state="roller_task"
    """로봇이 수행할 작업 ('ㄹ' 모양 및 너비 축소 적용)"""
    print("Performing roller task...")
    from DSR_ROBOT2 import posx, movel

    Y_CONST = -590
    ORIENTATION = [90, -90, 0]

    z_levels = [500, 450, 400, 300]
    X_MAX = 500
    X_MIN = -210

    print("Generating 'tapered-Z' path points...")
    path_points = []

    for i, z in enumerate(z_levels):
        if i % 2 == 0:
            path_points.append(posx([X_MAX, Y_CONST, z, *ORIENTATION]))
            path_points.append(posx([X_MIN, Y_CONST, z, *ORIENTATION]))
        else:
            path_points.append(posx([X_MIN, Y_CONST, z, *ORIENTATION]))
            path_points.append(posx([X_MAX, Y_CONST, z, *ORIENTATION]))

    print("--- Starting 'ㄹ' shape movement ---")

    for i, point in enumerate(path_points):
        print(f"Moving to point {i + 1}...")
        movel(point, vel=VELOCITY, acc=ACC)

        # ★ 각 선분 이동 끝날 때마다 긴급신호 체크
        if check_emergency():
            print("[ROLLER] 긴급 정지로 인한 조기 종료")
            return

    ready = posx([500, -590, 500, 90, -90, 0])
    movel(ready, vel=VELOCITY, acc=ACC)
    print("--- 'ㄹ' 모양 그리기 완료 ---")


def cloth_task():
    global state
    state="cloth_task"
    from DSR_ROBOT2 import (
        posx,
        movel,
        DR_FC_MOD_REL,
        wait,
        set_ref_coord,
        task_compliance_ctrl,
        set_desired_force,
    )
    from DSR_ROBOT2 import release_compliance_ctrl, release_force

    ready_pos = posx([500, -580, 500, 90, -90, 90])
    attach = posx([0, -40, 0, 0, 0, 0])
    detach = posx([500, -580, 300, 90, -90, 90])
    down = posx([0, 0, -200, 0, 0, 0])
    next_p = posx([-80, 0, 200, 0, 0, 0])

    movel(ready_pos, vel=VELOCITY, acc=ACC)

    for i in range(3):
        if check_emergency():
            print("[CLOTH] 긴급 정지로 인한 조기 종료")
            return

        set_ref_coord(1)
        task_compliance_ctrl([3000.00, 3000.00, 10.00, 200.00, 200.00, 200.00])
        wait(0.5)
        set_desired_force(
            fd=[0, 0, 5, 0, 0, 0],
            dir=[0, 0, 1, 0, 0, 0],
            time=0,
            mod=DR_FC_MOD_REL,
        )
        set_ref_coord(0)
        wait(0.5)
        movel(attach, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
        movel(down, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
        movel(detach, vel=VELOCITY, acc=ACC)
        detach[0] = detach[0] - 80
        wait(0.5)
        set_ref_coord(1)
        release_force(time=0.0)
        release_compliance_ctrl()
        set_ref_coord(0)
        movel(next_p, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)

    movel(ready_pos, vel=VELOCITY, acc=ACC)


def pickup_roller():
    global state
    state="pickup_roller"
    from DSR_ROBOT2 import posx, movel, wait
    from DSR_ROBOT2 import set_digital_output, DR_FC_MOD_REL

    roller = posx([524.98, 122.98, 363.38, 17.81, -178.75, -70.88])
    down = posx([0, 0, -30, 0, 0, 0])
    pickup = posx([0, 0, 192.34, 0, 0, 0])

    movel(roller, vel=VELOCITY, acc=ACC)
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    wait(1)
    movel(down, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
    set_digital_output(1, 1)
    set_digital_output(2, 1)
    wait(1)
    movel(pickup, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
    print("roller pick up complete")


def return_roller():
    global state
    state="return_roller"
    from DSR_ROBOT2 import posx, movel, wait
    from DSR_ROBOT2 import set_digital_output, DR_FC_MOD_REL

    roller = posx([524.98, 122.98, 563.38, 17.81, -178.75, -70.88])
    down = posx([0, 0, -230, 0, 0, 0])
    pickup = posx([0, 0, 192.34, 0, 0, 0])

    movel(roller, vel=VELOCITY, acc=ACC)
    movel(down, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    wait(1)
    movel(pickup, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
    print("roller return complete")


def pickup_cloth():
    global state
    state="pickup_cloth"
    from DSR_ROBOT2 import posx, movel, wait
    from DSR_ROBOT2 import set_digital_output, DR_FC_MOD_REL

    cloth = posx([610.68, -73.29, 325.61, 83.38, -179.74, -4.56])
    down = posx([0, 0, -70, 0, 0, 0])
    pickup = posx([0, 0, 115.16, 0, 0, 0])

    movel(cloth, vel=VELOCITY, acc=ACC)
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    wait(1)

    movel(down, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
    set_digital_output(1, 1)
    set_digital_output(2, 0)
    wait(1)
    movel(pickup, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
    print("cloth pick up complete")


def return_cloth():
    global state
    state="return_cloth"
    from DSR_ROBOT2 import posx, movel, wait
    from DSR_ROBOT2 import set_digital_output, DR_FC_MOD_REL

    cloth = posx([610.68, -73.29, 325.61, 83.38, -179.74, -4.56])
    down = posx([0, 0, -70, 0, 0, 0])
    pickup = posx([0, 0, 115.16, 0, 0, 0])

    movel(cloth, vel=VELOCITY, acc=ACC)
    movel(down, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    wait(1)
    movel(pickup, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
    print("cloth return complete")


def perform_task():
    """로봇이 수행할 전체 작업 (한 사이클)"""
    print("Performing full task...")

    from DSR_ROBOT2 import posx, movej, movel, DR_FC_MOD_REL

    home = [0, 0, 90, 0, 90, 0]
    up = posx([0, 0, 100, 0, 0, 0])
    safe = posx([150, 0, 0, 0, 0, 0])

    movej(home, vel=VELOCITY, acc=ACC)
    if check_emergency():
        return
    if stoped==False:
        movel(up, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
        if check_emergency():
            return
    if stoped==False:
        movel(safe, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
        if check_emergency():
            return
    if stoped==False:
        pickup_roller()
        if check_emergency():
            return
    if stoped==False:
        roller_task()
        if check_emergency():
            return
    if stoped==False:
        return_roller()
        if check_emergency():
            return
    if stoped==False:
        pickup_cloth()
        if check_emergency():
            return
    if stoped==False:
        cloth_task()
        if check_emergency():
            return
    if stoped==False:
        return_cloth()
        if check_emergency():
            return


def main(args=None):
    """메인 함수: 로봇 동작 + '50' 명령 수신"""
    global latest_cmd

    rclpy.init(args=args)

    # 로봇용 노드 (DSR_ROBOT2에서 사용하는 노드)
    robot_node = rclpy.create_node("move_basic", namespace=ROBOT_ID)
    DR_init.__dsr__node = robot_node  # 반드시 DSR_ROBOT2 import 전에 설정

    # 명령 수신 전용 노드
    cmd_node = rclpy.create_node("move_basic_cmd", namespace=ROBOT_ID)

    # 명령 콜백: 토픽 값 저장
    def cmd_callback(msg: String):
        latest_cmd["data"] = msg.data.strip()
        print(f"[CMD] 받은 값(raw): [{msg.data}] -> 저장값: [{latest_cmd['data']}]")

    cmd_node.create_subscription(
        String,
        "task_command",  # 실제 토픽: /dsr01/task_command
        cmd_callback,
        10,
    )

    # cmd_node 전용 executor + spin 스레드
    executor = SingleThreadedExecutor()
    executor.add_node(cmd_node)

    def spin_worker():
        try:
            executor.spin()
        except Exception as e:
            print(f"[EXECUTOR] error: {e}")

    spin_thread = threading.Thread(target=spin_worker, daemon=True)
    spin_thread.start()

    try:
        initialize_robot()

        # 필요하면 while rclpy.ok(): 로 반복 처리도 가능
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        executor.shutdown()
        cmd_node.destroy_node()
        robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
