import os
from ament_index_python.resources import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, Int32
import rclpy


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

        font = QFont()
        font.setPointSize(16)   # 글자 크기
        font.setBold(True)      # 볼드 처리
        self.label_state.setFont(font)
        self.label_state.setAlignment(Qt.AlignCenter)  # 가운데 정렬

        # ROS2 QoS
        qos = QoSProfile(depth=1)

        # ROS2 Publishers
        self.pub_start = self.node.create_publisher(Int32, '/dsr01/task/start', qos)
        self.pub_end = self.node.create_publisher(Bool, 'task/end', qos)
        self.pub_stop = self.node.create_publisher(Bool, 'task/emergency_stop', qos)
        self.node.get_logger().info('ROS 2 Task control publishers initialized.')

        # progressBar 초기화
        self.progressBar.setValue(0)

        # ROS2 Subscriber (progress 값 수신)
        self.sub_progress = self.node.create_subscription(
            Int32,
            'task/progress',
            self.progress_callback,
            qos
        )

        # 상태 관리
        self.state = {'start': False, 'end': False, 'estop': False}

        # UI 상태 레이블 초기화
        self.label_state.setText("Waiting for command...")

        # 버튼 이벤트 연결
        self.push_button_start.clicked.connect(self.on_start_clicked)
        self.push_button_end.clicked.connect(self.on_end_clicked)
        self.push_button_estop.clicked.connect(self.on_estop_clicked)

    def reset_states(self):
        """모든 상태를 False로 초기화"""
        self.state = {key: False for key in self.state}

    def on_start_clicked(self):
        self.reset_states()
        self.state['start'] = True

        # 체크박스 상태 확인
        roller_checked = self.checkBox_roller.isChecked()
        cloth_checked = self.checkBox_cloth.isChecked()

        # 메시지 값 결정
        if roller_checked and cloth_checked:
            msg_value = 3
            log_msg = "Both roller and cloth selected."
            ui_state_msg = "Task started: Roller + Cloth"
        elif roller_checked:
            msg_value = 1
            log_msg = "Roller selected."
            ui_state_msg = "Task started: Roller"
        elif cloth_checked:
            msg_value = 2
            log_msg = "Cloth selected."
            ui_state_msg = "Task started: Cloth"
        else:
            msg_value = 0
            log_msg = "No task selected."
            ui_state_msg = "No task selected."

        # Int32 메시지 발행
        self.pub_start.publish(Int32(data=msg_value))
        self.node.get_logger().info(f"START clicked: {log_msg} (Int32={msg_value})")

        # 상태 UI 업데이트
        self.label_state.setText(ui_state_msg)

        # 나머지 상태 Bool 발행
        self.pub_end.publish(Bool(data=self.state['end']))
        self.pub_stop.publish(Bool(data=self.state['estop']))

    def on_end_clicked(self):
        self.reset_states()
        self.state['end'] = True

        self.pub_end.publish(Bool(data=True))
        self.pub_start.publish(Int32(data=0))
        self.pub_stop.publish(Bool(data=self.state['estop']))
        self.node.get_logger().info('State set: END')

        # UI 상태 업데이트
        self.label_state.setText("Task ended.")

    def on_estop_clicked(self):
        self.reset_states()
        self.state['estop'] = True

        self.pub_stop.publish(Bool(data=True))
        self.pub_start.publish(Int32(data=0))
        self.pub_end.publish(Bool(data=self.state['end']))
        self.node.get_logger().info('State set: EMERGENCY STOP')

        # UI 상태 업데이트
        self.label_state.setText("EMERGENCY STOP triggered!")

    def progress_callback(self, msg: Int32):
        """Int형 메시지를 받아 progressBar 업데이트"""
        value = max(0, min(100, msg.data))
        self.progressBar.setValue(value)
        self.node.get_logger().info(f'Progress updated: {value}%')

        # Optional: 진행 중 상태 메시지 표시
        if self.state['start']:
            self.label_state.setText(f"Working... {value}%")
