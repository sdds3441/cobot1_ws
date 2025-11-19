import rclpy
import threading
import time
import sys
from std_msgs.msg import Int32, String # [수정] String 메시지 임포트
from rclpy.executors import SingleThreadedExecutor 
from rclpy.qos import QoSProfile 

import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight" 
ROBOT_TOOL = "GripperDA_v1"  

# 이동 속도 및 가속도
VELOCITY = 500
ACC = 100

# ===============================================
# [수정] 글로벌 상태 변수 (체크포인트 및 제어)
# ===============================================
latest_cmd = 0       
active_task_cmd = 0  
current_step_index = 0 
task_thread = None   

# 스레드 간 안전한 진행도 공유를 위한 변수
shared_progress = 0
progress_lock = threading.Lock()

# [추가] 현재 작업 상태 문자열
shared_status = "IDLE" 

# [수정] 최종 100% 발행을 메인 루프에 알리는 플래그
final_publish_needed = False 

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# [추가] CMD -> 작업 이름 매핑 딕셔너리
TASK_NAME_MAP = {
    1: "ROLLER_TASK",
    2: "CLOTH_TASK",
    3: "FULL_TASK"
}


# ===============================================
# 안전 이동 헬퍼 함수: 모든 로봇 이동을 감싸는 함수
# ===============================================

def safe_move(move_func, *args, **kwargs):
    """
    모든 로봇 이동 함수를 감싸는 헬퍼 함수.
    latest_cmd == 0 일 때 DSR stop()으로 인한 예외를 처리합니다.
    """
    global latest_cmd
    
    # 이동 명령 전에 중단 조건 확인 (0이면 예외 발생)
    if latest_cmd == 0 and active_task_cmd != 0:
        raise Exception("Task Interrupted by latest_cmd = 0 flag check.")
        
    try:
        # 로봇 이동 실행 (블로킹)
        move_func(*args, **kwargs)
        
    except Exception as e:
        if latest_cmd == 0:
            # DSR stop() 호출로 인해 발생한 예외는 깨끗한 중단으로 처리
            raise Exception("Interrupted by DSR stop() command.")
        else:
            # 그 외의 심각한 오류는 다시 발생
            raise e


# ===============================================
# 콜백 함수: latest_cmd 업데이트 및 강제 중단/재개
# ===============================================

def cmd_callback(msg):
    global latest_cmd, active_task_cmd, current_step_index, task_thread, shared_progress, progress_lock, shared_status
    
    new_cmd = msg.data
    
    # 1. New Task (1, 2, 3): 새 작업 시작
    if new_cmd in [1, 2, 3]:
        # 이미 작업 중이면 무시 (재시작하려면 먼저 0을 보내야 함)
        if task_thread is not None and task_thread.is_alive() and latest_cmd != new_cmd:
             print("경고: 이미 다른 작업이 진행 중입니다. 먼저 0을 보내 중단하세요.")
             return
             
        active_task_cmd = new_cmd
        current_step_index = 0 # 새 작업은 0부터 시작
        latest_cmd = new_cmd
        
        # [수정] 새 작업 시작 시, shared_progress를 0으로 설정하여 프로그레스바 리셋 유도
        with progress_lock:
             shared_progress = 0
             # FULL_TASK의 경우, 시작 시점에 ROLLER_PHASE를 명시
             if new_cmd == 3:
                 shared_status = "FULL_TASK_ROLLER_PHASE_RUNNING"
             else:
                 shared_status = TASK_NAME_MAP.get(new_cmd, "UNKNOWN_TASK") + "_RUNNING"
        
    # 2. Pause (0): 일시 정지
    elif new_cmd == 0 and active_task_cmd != 0 and latest_cmd != 0:
        print(f"🚨 PAUSE 명령 수신! Task {active_task_cmd} 일시 정지.")
        latest_cmd = 0 # 메인 루프에 정지 신호 전달
        
        # [수정] Pause 상태 업데이트: 현재 RUNNING 상태를 PAUSED로 변경
        with progress_lock:
             if shared_status.endswith("_RUNNING"):
                 # _RUNNING을 제거하고 _PAUSED로 대체
                 shared_status = shared_status[:-8] + "_PAUSED" 
             else:
                 shared_status = shared_status + "_PAUSED" # 안전 장치
             
        try:
            from DSR_ROBOT2 import stop
            stop() 
        except Exception:
            pass
            
    # 3. Resume (4): 중단된 작업 재개 
    elif new_cmd == 4 and active_task_cmd != 0 and latest_cmd == 0:
        # 일시 정지 상태(latest_cmd=0)이고, 이전에 시작된 작업이 있을 때만 재개 가능
        if current_step_index > 0:
            print(f"🔄 RESUME 명령 수신! Task {active_task_cmd} 재개 신호 (Index: {current_step_index}).")
            # latest_cmd를 원래 active_task_cmd로 복원하여 메인 루프가 스레드를 다시 시작하도록 유도
            latest_cmd = active_task_cmd 
            
            # [수정] Resume 시 상태를 다시 PAUSED에서 RUNNING으로 변경
            with progress_lock:
                 if shared_status.endswith("_PAUSED"):
                     shared_status = shared_status[:-7] + "_RUNNING"
                 else:
                     shared_status = shared_status + "_RUNNING" # 안전 장치

        else:
             print("경고: Resume 명령이 들어왔으나 저장된 진행 상태가 없습니다.")


    # 4. 기타 명령: 무시


# ===============================================
# 로봇 초기화 및 하위 작업 함수
# (생략: 변경 없음)
# ===============================================

def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp 
    set_tool(ROBOT_TCP); set_tcp(ROBOT_TOOL)
    print("#"*50); print("Initializing robot with the following settings:"); print(f"ROBOT_ID: {ROBOT_ID}"); print(f"VELOCITY: {VELOCITY}"); print("#"*50)

def move_to_safe_pos():
    from DSR_ROBOT2 import posx, movej, movel, DR_FC_MOD_REL
    home = [0, 0, 90, 0, 90, 0]; up = posx([0,0,100,0,0,0]); safe_pos = posx([150,0,0,0,0,0])
    safe_move(movej, home, vel=VELOCITY, acc=ACC); safe_move(movel, up, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
    safe_move(movel, safe_pos, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL); print("초기 준비 자세 이동 완료.")


def roller_move_p1(): 
    from DSR_ROBOT2 import posx, movel
    Y_CONST = -440; ORIENTATION = [90, -90, 0]; z = 450; X_MAX = 520
    safe_move(movel, posx([X_MAX, Y_CONST, z, *ORIENTATION]), vel=VELOCITY, acc=ACC); print("Roller P1: (500, 500) 이동 완료.")
def roller_move_p2():
    from DSR_ROBOT2 import posx, movel
    Y_CONST = -440; ORIENTATION = [90, -90, 0]; z = 450; X_MIN = 0
    safe_move(movel, posx([X_MIN, Y_CONST, z, *ORIENTATION]), vel=VELOCITY, acc=ACC); print("Roller P2: (-210, 500) 이동 완료.")
def roller_move_p3():
    from DSR_ROBOT2 import posx, movel
    Y_CONST = -440; ORIENTATION = [90, -90, 0]; z = 350; X_MIN = 0
    safe_move(movel, posx([X_MIN, Y_CONST, z, *ORIENTATION]), vel=VELOCITY, acc=ACC); print("Roller P3: (-210, 450) 이동 완료.")
def roller_move_p4():
    from DSR_ROBOT2 import posx, movel
    Y_CONST = -440; ORIENTATION = [90, -90, 0]; z = 350; X_MAX = 520
    safe_move(movel, posx([X_MAX, Y_CONST, z, *ORIENTATION]), vel=VELOCITY, acc=ACC); print("Roller P4: (500, 450) 이동 완료.")
def roller_move_p5():
    from DSR_ROBOT2 import posx, movel
    Y_CONST = -440; ORIENTATION = [90, -90, 0]; z = 250; X_MAX = 520
    safe_move(movel, posx([X_MAX, Y_CONST, z, *ORIENTATION]), vel=VELOCITY, acc=ACC); print("Roller P5: (500, 400) 이동 완료.")
def roller_move_p6():
    from DSR_ROBOT2 import posx, movel
    Y_CONST = -440; ORIENTATION = [90, -90, 0]; z = 250; X_MIN = 0
    safe_move(movel, posx([X_MIN, Y_CONST, z, *ORIENTATION]), vel=VELOCITY, acc=ACC); print("Roller P6: (-210, 400) 이동 완료.")
# def roller_move_p7():
#     from DSR_ROBOT2 import posx, movel
#     Y_CONST = -440; ORIENTATION = [90, -90, 0]; z = 300; X_MIN = 0
#     safe_move(movel, posx([X_MIN, Y_CONST, z, *ORIENTATION]), vel=VELOCITY, acc=ACC); print("Roller P7: (-210, 300) 이동 완료.")
# def roller_move_p8():
#     from DSR_ROBOT2 import posx, movel
#     Y_CONST = -440; ORIENTATION = [90, -90, 0]; z = 300; X_MAX = 520
#     safe_move(movel, posx([X_MAX, Y_CONST, z, *ORIENTATION]), vel=VELOCITY, acc=ACC); print("Roller P8: (500, 300) 이동 완료.")
def roller_move_final_ready():
    from DSR_ROBOT2 import posx, movel
    ready=posx([500,-400,500,90,-90,0]); safe_move(movel, ready, vel=VELOCITY,acc=ACC); print("--- 'ㄹ' 모양 그리기 완료 및 최종 대기 자세 이동 ---")
def cloth_task():
    from DSR_ROBOT2 import posx,movel,DR_FC_MOD_REL, wait, set_ref_coord,task_compliance_ctrl,set_desired_force, release_compliance_ctrl, release_force
    ready_pos = posx([500, -500, 500, 90, -90, 90]); attach=posx([0,-20,0,0,0,0]); down = posx([0,0,-300,0,0,0]); 
    safe_move(movel, ready_pos, vel=VELOCITY, acc=ACC)
    for i in range(1,8):    
        task_compliance_ctrl([500.00, 200.00, 10.00, 10.00, 10.00, 10.00]); wait(0.2) 
        safe_move(movel, attach, vel=VELOCITY,acc=ACC,mod=DR_FC_MOD_REL); safe_move(movel, down, vel=VELOCITY, acc=ACC, mod=DR_FC_MOD_REL)
        release_compliance_ctrl(); 
        if i<7:
            next_pos=posx([ready_pos[0]-110*i,-480,500,90,-90,90])   
            safe_move(movel, next_pos, vel=VELOCITY, acc=ACC)
    safe_move(movel, posx([500, -480, 500, 90, -90, 90]), vel=VELOCITY, acc=ACC)
def pickup_roller():
    from DSR_ROBOT2 import posx,movel,wait, set_digital_output, DR_FC_MOD_REL
    roller= posx([546.98, 122.98, 363.38, 17.81, -178.75, -70.88]); down=posx([0,0,-30,0,0,0,]); pickup=posx([0, 0, 150, 0, 0, 0]); safe_spot=posx([0,-100,100,0,0,0])
    safe_move(movel, roller, vel=VELOCITY,acc=ACC); set_digital_output(1,0); set_digital_output(2,1); wait(1)
    safe_move(movel, down, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL); set_digital_output(1,1); set_digital_output(2,1); wait(1)
    safe_move(movel, pickup, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL); safe_move(movel, safe_spot, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL);print('roller pick up complete')
def return_roller():
    from DSR_ROBOT2 import posx,movel,wait, set_digital_output, DR_FC_MOD_REL
    roller= posx([546.98, 122.98, 363.38, 17.81, -178.75, -70.88]); down=posx([0,0,-30,0,0,0,]); pickup=posx([0, 0, 142.34, 0, 0, 0]); safe_spot=posx([566.98,22.98,400.38,17.81, -178.75, -70.88])
    safe_move(movel, safe_spot, vel=VELOCITY,acc=ACC); safe_move(movel, roller, vel=VELOCITY,acc=ACC); safe_move(movel, down, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    set_digital_output(1,0); set_digital_output(2,1); wait(1); safe_move(movel, pickup, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    print('roller return complete')
def pickup_cloth():
    from DSR_ROBOT2 import posx,movel,wait, set_digital_output, DR_FC_MOD_REL
    cloth = posx([461, -73.29, 325.61, 83.38, -179.74, -4.56]); down=posx([0,0,-70,0,0,0,]); pickup=posx([0, 0, 115.16, 0, 0, 0])
    safe_move(movel, cloth, vel=VELOCITY,acc=ACC); set_digital_output(1,0); set_digital_output(2,1); wait(1)
    safe_move(movel, down, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL); set_digital_output(1,1); set_digital_output(2,0); wait(1)
    safe_move(movel, pickup, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL); print('cloth pick up complete')
def return_cloth():
    from DSR_ROBOT2 import posx,movel,wait, set_digital_output, DR_FC_MOD_REL
    cloth = posx([461, -73.29, 325.61, 83.38, -179.74, -4.56]); down=posx([0,0,-70,0,0,0,]); pickup=posx([0, 0, 115.16, 0, 0, 0])
    safe_move(movel, cloth, vel=VELOCITY,acc=ACC); safe_move(movel, down, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    set_digital_output(1,0); set_digital_output(2,1); wait(1); safe_move(movel, pickup, vel=VELOCITY,acc=ACC, mod=DR_FC_MOD_REL)
    print('cloth return complete')

def task_sequence_roller():
    return [
        move_to_safe_pos, pickup_roller, roller_move_p1, roller_move_p2, roller_move_p3, roller_move_p4, roller_move_p5, roller_move_p6, roller_move_final_ready, return_roller
    ]
def task_sequence_cloth():
    return [
        move_to_safe_pos, pickup_cloth, cloth_task, return_cloth
    ]
def task_sequence_full():
    return [
        move_to_safe_pos, pickup_roller, roller_move_p1, roller_move_p2, roller_move_p3, roller_move_p4, roller_move_p5, roller_move_p6, roller_move_final_ready, return_roller, pickup_cloth, cloth_task, return_cloth
    ]
TASK_CMD_MAP = {
    1: task_sequence_roller(), 2: task_sequence_cloth(), 3: task_sequence_full()
}
    
    
# ===============================================
# 메인 작업 로직 (체크포인트 관리)
# ===============================================

def perform_task_logic(cmd):
    """
    cmd 값(1, 2, 3)에 따라 작업을 순차적으로 수행하며, current_step_index를 관리합니다.
    """
    global latest_cmd, active_task_cmd, current_step_index, shared_progress, progress_lock, final_publish_needed, shared_status
    
    task_steps = TASK_CMD_MAP.get(cmd)
    if not task_steps:
        print(f"작업 ID {cmd}에 해당하는 작업 목록이 없습니다.")
        latest_cmd = 0
        active_task_cmd = 0
        with progress_lock: shared_status = "ERROR"
        return
        
    task_completed = False
    start_index = current_step_index
    total_steps = len(task_steps) 
    task_name = TASK_NAME_MAP.get(cmd, "UNKNOWN_TASK")

    try:
        # [수정] 작업 스레드 시작 시 상태를 RUNNING으로 재확인
        with progress_lock:
             shared_status = task_name + "_RUNNING" 
             
        # 하위 작업을 순차적으로 실행
        for i in range(start_index, total_steps):
            
            # 1. 실행 전 중단/취소 명령 확인
            if latest_cmd == 0:
                print(f"🛑 perform_task: Step {i+1} 실행 전 일시 정지 (CMD=0). Index {i} 저장됨.")
                current_step_index = i # 다음 실행할 step 저장
                return
            
            # 2. 하위 작업 실행
            step_function = task_steps[i]
            print(f"▶ Step {i+1}/{total_steps}: {step_function.__name__} 실행 중...")
            step_function()
            
            # 3. 작업 성공 후 체크포인트 업데이트 및 진행도 발행 (스레드 안전)
            current_step_index = i + 1 
            
            # [수정] 진행도 계산 및 Lock을 사용하여 전역 변수 업데이트
            progress_percent = int((current_step_index / total_steps) * 100)
            
            # [추가] FULL_TASK 세부 단계 구분
            phase_suffix = ""
            if cmd == 3:
                # 0 ~ 11: Roller (총 10단계)
                if current_step_index <= 10: 
                    phase_suffix = "_ROLLER_PHASE"
                # 13 ~ 15: Cloth (나머지 단계)
                elif current_step_index > 10:
                    phase_suffix = "_CLOTH_PHASE"

            with progress_lock:
                 shared_progress = progress_percent
                 # [수정] 상태 문자열에 세부 단계 추가 (예: FULL_TASK_ROLLER_PHASE_RUNNING)
                 shared_status = task_name + phase_suffix + "_RUNNING"
            print(f"--- PROGRESS SHARED: {progress_percent}% ({current_step_index}/{total_steps}) ---")


        task_completed = True

    except Exception as e:
        if "Interrupted by DSR stop()" in str(e) or "Interrupted by latest_cmd" in str(e):
            # 중단 시에도 다음 스텝 인덱스를 정확히 저장합니다.
            current_step_index = i
            # [수정] Pause 상태 설정: 현재 shared_status에서 _RUNNING을 _PAUSED로 변경
            with progress_lock:
                if shared_status.endswith("_RUNNING"):
                    shared_status = shared_status[:-8] + "_PAUSED"
                else:
                    shared_status = shared_status + "_PAUSED"
            print(f"🛑 perform_task: 강제 중단 명령으로 인해 작업을 종료합니다. (Index {current_step_index} 유지)")
            
        else:
            print(f"⚠️ perform_task: 예상치 못한 오류로 작업을 중단합니다: {e}")
            with progress_lock:
                shared_status = task_name + "_ERROR"
            
    finally:
        # 작업 완료 또는 중단 후 상태 정리
        if task_completed:
            print("✅ 작업 성공적으로 완료. 상태 리셋.")
            
            # [핵심 수정] 100% Lock을 사용하여 업데이트 및 플래그 설정
            with progress_lock:
                shared_progress = 100
                final_publish_needed = True # 메인 루프에 최종 발행 신호 전송
                shared_status = task_name + "_COMPLETED" # 완료 상태 설정
                
            current_step_index = 0
            active_task_cmd = 0
            latest_cmd = 0
        elif latest_cmd == 0:
            # 일시 정지 상태로 종료된 경우: Index와 active_task_cmd는 유지됨. latest_cmd는 0 유지.
            # shared_status는 이미 PAUSED로 설정됨.
            print(f"작업 스레드 종료. 다음 재개 지점 (Index: {current_step_index})")
        else:
            # 실패(Fatal Error)로 종료된 경우: 상태 리셋.
            print("❌ 심각한 오류로 인해 작업 및 상태를 리셋합니다.")
            current_step_index = 0
            active_task_cmd = 0
            latest_cmd = 0
            # shared_status는 이미 ERROR로 설정됨


# ===============================================
# 메인 함수: 비동기 실행 관리 (수정된 부분)
# ===============================================

def main(args=None):
    global latest_cmd, task_thread, active_task_cmd, current_step_index, shared_progress, progress_lock, final_publish_needed, shared_status

    rclpy.init(args=args)
    
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

    # ROS2 QoS
    qos = QoSProfile(depth=1)
    
    # Progress Publisher 초기화: 절대 경로 사용
    progress_pub = node.create_publisher(Int32, '/task/progress', qos) 
    # [추가] Status Publisher 초기화: 절대 경로 사용
    status_pub = node.create_publisher(String, '/task/status', qos) 

    # Subscription 설정
    node.create_subscription(Int32, "/dsr01/task/start", cmd_callback, 1) 
    
    # DSR_ROBOT2 초기화
    DR_init.__dsr__node = node
    initialize_robot()

    print("대기 중... '/task/start'에서 0이 아닌 값이 들어오면 작업을 시작합니다.")
    print("CMD: 1, 2, 3 = 새 작업 시작 / 0 = 일시 정지 / 4 = 이어하기")

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    # Progress Publisher를 위한 타이머 콜백 정의
    def progress_timer_callback():
        global shared_progress, progress_lock, task_thread, shared_status
        current_progress = 0
        current_status = "IDLE"
        
        # 락을 사용하여 공유 변수를 안전하게 읽습니다.
        with progress_lock:
            current_progress = shared_progress
            current_status = shared_status
            
        # 스레드가 살아있거나, progress가 0보다 크거나, 100%일 때 발행
        if current_progress > 0 or (task_thread is not None and task_thread.is_alive()) or current_progress == 100 or current_status != "IDLE":
            # [수정] 진행도 및 상태 발행
            progress_pub.publish(Int32(data=current_progress))
            status_pub.publish(String(data=current_status))
            # print(f"Published Progress: {current_progress}, Status: {current_status}") 

    # 타이머 생성: 100ms마다 발행 시도
    timer_period = 0.1  # seconds
    node.create_timer(timer_period, progress_timer_callback)


    while rclpy.ok():
        # 1. 짧게 스핀하여 콜백(cmd_callback, progress_timer_callback)을 처리
        executor.spin_once(timeout_sec=0.1)
        
        # 2. 100% 최종 발행 보장 로직
        if final_publish_needed:
            # 작업 스레드가 100%를 설정한 후, 정리되기 전에 메인 스레드에서 강제 발행
            progress_pub.publish(Int32(data=100))
            status_pub.publish(String(data=shared_status)) # 최종 완료 상태 발행
            final_publish_needed = False
            print("MAIN LOOP: Forced 100% final progress publish.")

        # 3. 작업이 완료되면 스레드 객체를 정리 (새 스레드 시작 전에 처리되어야 함)
        if task_thread is not None and not task_thread.is_alive():
            task_thread.join()
            task_thread = None
            
            # [수정] 100% 발행 후 shared_progress와 shared_status를 완전히 0/IDLE로 리셋
            with progress_lock:
                 shared_progress = 0
                 shared_status = "IDLE" 
                 
            print("MAIN LOOP: Dead task thread cleaned up (task_thread = None).") 

        # 4. 명령이 들어왔고 (latest_cmd != 0), 현재 작업 중이 아닐 때 (task_thread is None) 스레드 시작
        if latest_cmd != 0 and task_thread is None:
            
            cmd_to_execute = latest_cmd
            
            # 재개 명령이었는지 확인하고 로그 출력
            if current_step_index > 0 and cmd_to_execute == active_task_cmd:
                print(f"MAIN LOOP: Resuming Task ({cmd_to_execute}) from Index: {current_step_index}.")
            else: 
                # 새 작업 시작 (current_step_index는 0)
                print(f"MAIN LOOP: Starting New Task ({cmd_to_execute}) from Index: {current_step_index}.")
                
            # 스레드를 생성하고 로봇 동작 로직을 비동기적으로 실행
            task_thread = threading.Thread(target=perform_task_logic, args=(cmd_to_execute,))
            task_thread.start()
            

    # 프로그램 종료 전 스레드 정리
    if task_thread is not None and task_thread.is_alive():
        print("프로그램 종료 전 작업 스레드 대기...")
        task_thread.join()
        
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()