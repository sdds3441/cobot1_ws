import os
from ament_index_python.resources import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32, String # [수정] String 임포트 추가
import rclpy

# 0 : 작업 중지, 1 : 롤러 작업, 2 : 걸레 작업, 3 : 모든 작업, 4 : 작업 재개, 5 : 작업 선택되지 않음

class ExamplesWidget(QWidget):

    def __init__(self, node):
        super().__init__()
        self.node = node

        # UI 로드
        pkg_name = 'rqt_example'
        ui_filename = 'rqt_example.ui'
        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)
        
        self.setWindowTitle("sky cleaner")  # [추가] RQT 윈도우 타이틀을 "sky cleaner"로 설정

        font = QFont()
        font.setPointSize(16)   # 글자 크기
        font.setBold(True)      # 볼드 처리
        self.label_state.setFont(font)
        self.label_state.setAlignment(Qt.AlignCenter)  # 가운데 정렬
        self.label_state.setWordWrap(True)  # [수정] 긴 텍스트 자동 줄바꿈 활성화
        
        # [추가] 체크박스 폰트 크기 조정
        # .ui 파일에 정의된 이름(checkBox_roller, checkBox_cloth)을 사용하여 폰트 적용
        try:
            self.checkBox_roller.setFont(font)
            self.checkBox_cloth.setFont(font)
        except AttributeError:
             # .ui 파일에 해당 이름의 체크박스가 없는 경우를 대비
             self.node.get_logger().warn("Checkboxes not found in .ui file (e.g., checkBox_roller). Skipping font change.")


        # ROS2 QoS
        qos = QoSProfile(depth=1)

        # ROS2 Publishers
        self.pub_start = self.node.create_publisher(Int32, '/dsr01/task/start', qos)
        self.node.get_logger().info('ROS 2 Task control publishers initialized.')

        # progressBar 초기화
        self.progressBar.setValue(0)

        # [추가] UI 상태 저장 변수 초기화
        self.progress_value = 0
        self.status_message = "IDLE"

        # ROS2 Subscriber (progress 값 수신)
        self.sub_progress = self.node.create_subscription(
            Int32,
            '/task/progress', # [수정] 절대 경로 사용 (로봇 노드와 일치)
            self.progress_callback,
            qos
        )
        
        # [추가] ROS2 Subscriber (status 값 수신)
        self.sub_status = self.node.create_subscription(
            String,
            '/task/status',
            self.status_callback,
            qos
        )
        self.node.get_logger().info('ROS 2 Task status subscribers initialized.')


        # 상태 관리 (UI에서 클릭 시작 여부만 판단)
        self.state = {'start_clicked': False}

        # UI 상태 레이블 초기화
        self.label_state.setText("Waiting for command...")

        # 버튼 이벤트 연결
        self.push_button_start.clicked.connect(self.on_start_clicked)
        # self.push_button_end.clicked.connect(self.on_end_clicked) # push_button_end가 없으므로 연결 제거
        self.push_button_estop.clicked.connect(self.on_estop_clicked)
        self.push_button_continue.clicked.connect(self.on_start_continue_clicked)

    def _format_status(self, status):
        """상태 문자열에서 접미사를 제거하고 줄바꿈 및 공백을 추가하여 가독성을 높입니다."""
        s = status
        
        # 1. 상태 접미사 제거 (COMPLETED, STOP, RUNNING, ERROR)
        if s.endswith("_COMPLETED"):
            s = s.replace("_COMPLETED", "")
        elif s.endswith("_STOP"):
            s = s.replace("_STOP", "")
        elif s.endswith("_RUNNING"):
            s = s.replace("_RUNNING", "")
        elif s.endswith("_ERROR"):
            s = s.replace("_ERROR", "")
        
        # 2. FULL_TASK의 PHASE 구분을 위한 줄바꿈 추가
        s = s.replace("FULL_TASK_ROLLER_PHASE", "FULL TASK\n(ROLLER PHASE)")
        s = s.replace("FULL_TASK_CLOTH_PHASE", "FULL TASK\n(CLOTH PHASE)")
        
        # 3. ROLLER/CLOTH TASK 구분을 위한 줄바꿈 추가
        s = s.replace("ROLLER_TASK", "ROLLER TASK")
        s = s.replace("CLOTH_TASK", "CLOTH TASK")
        
        # 4. 남아있는 언더바를 공백으로 변환하여 가독성 확보
        s = s.replace("_", " ") 
        
        return s

    def update_ui_label(self):
        """진행도와 상태 메시지를 통합하여 UI 라벨을 업데이트"""
        progress = self.progress_value
        status = self.status_message
        
        # 상태 메시지를 가독성 높게 포맷
        task_info = self._format_status(status)
        
        # 상태 접미대에 따른 최종 디스플레이 텍스트 구성
        if status == "IDLE":
            display_text = "Waiting for command..."
        elif status.endswith("_COMPLETED"):
            # 완료 상태는 두 줄로 명확하게 표시
            display_text = f"✅ COMPLETED:\n{task_info} ({progress}%)"
        elif status.endswith("_STOP"):
            # 일시 정지 상태는 두 줄로 명확하게 표시 (로봇 노드의 _STOP를 사용)
            display_text = f"⏸ STOP:\n{task_info} ({progress}%)"
        elif status.endswith("_ERROR"):
             display_text = f"❌ ERROR:\n{task_info} ({progress}%)"
        else: # RUNNING 상태 (status.endswith("_RUNNING") 또는 기타)
            # 실행 중 상태는 두 줄로 명확하게 표시
            display_text = f"▶ RUNNING:\n{task_info} ({progress}%)"
            
        self.label_state.setText(display_text)
        self.progressBar.setValue(progress)


    def reset_states(self):
        """모든 상태를 False로 초기화"""
        self.state = {'start_clicked': False}

    def on_start_clicked(self):
        self.reset_states()
        self.state['start_clicked'] = True

        roller_checked = self.checkBox_roller.isChecked()
        cloth_checked = self.checkBox_cloth.isChecked()

        # 메시지 값 결정
        if roller_checked and cloth_checked:
            msg_value = 3
        elif roller_checked:
            msg_value = 1
        elif cloth_checked:
            msg_value = 2
        else:
            msg_value = 5

        # Int32 메시지 발행
        self.pub_start.publish(Int32(data=msg_value))
        self.node.get_logger().info(f"START clicked: Int32={msg_value}")
        
        # 초기 UI 상태는 로봇 노드에서 발행되는 상태로 덮어씌워짐

    # def on_end_clicked(self):
    #     self.reset_states()
    #     # CMD=0 (Pause/End)을 사용하여 모든 동작 중지
    #     self.pub_start.publish(Int32(data=0))
    #     self.node.get_logger().info('END clicked: Sent CMD 0 (Pause/Stop)')
    #     # UI 상태는 로봇 노드에서 STOP 또는 IDLE 메시지를 받아 업데이트됨

    def on_start_continue_clicked(self):
        """사용자가 입력한 정수를 읽고 ROS2 토픽으로 발행"""
        try:
            # QLineEdit에서 문자열 읽고 정수 변환
            user_input = int(self.lineEdit_input.text())
        except ValueError:
            self.node.get_logger().warn("유효한 정수를 입력하세요!")
            return
        roller_checked = self.checkBox_roller.isChecked()
        cloth_checked = self.checkBox_cloth.isChecked()

        if roller_checked:
            msg_value = user_input +100
        elif cloth_checked:
            msg_value= user_input + 200
        
        self.pub_start.publish(Int32(data=msg_value))
        self.node.get_logger().info(f"CONTINUE clicked: Int32= {msg_value}")

    def on_estop_clicked(self):
        self.reset_states()
        # E-Stop은 강제 중단 CMD=0을 보냄
        self.pub_start.publish(Int32(data=0))
        self.node.get_logger().info('EMERGENCY STOP clicked: Sent CMD 0')

    def progress_callback(self, msg: Int32):
        """Int형 메시지를 받아 진행도 값을 저장하고 UI 업데이트 요청"""
        self.progress_value = max(0, min(100, msg.data))
        self.update_ui_label()

    def status_callback(self, msg: String):
        """String형 메시지를 받아 상태 메시지를 저장하고 UI 업데이트 요청"""
        self.status_message = msg.data
        self.update_ui_label()