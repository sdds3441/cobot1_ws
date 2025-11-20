import os
from ament_index_python.resources import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QGraphicsView, QVBoxLayout 
from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QFont, QPainter, QPen, QColor, QBrush, QTransform 
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32, String, Float64MultiArray 
import rclpy
from collections import deque


# =========================================================================
# 1. 좌표 시각화를 위한 커스텀 GraphicsView
# =========================================================================

class CoordinateGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        # self.points = []
        self.points = deque(maxlen=200)
        self.setRenderHint(QPainter.Antialiasing)
        self.setBackgroundBrush(QBrush(QColor(240, 240, 240)))

        self.X_MIN = -0.7
        self.X_MAX = 0.0
        self.Z_MIN_PLOT = -0.55
        self.Z_MAX_PLOT = 0.05

        self.VIEW_X_RANGE = self.X_MAX - self.X_MIN
        self.VIEW_Z_RANGE = self.Z_MAX_PLOT - self.Z_MIN_PLOT

        # 선을 그릴 최소 거리 기준
        self.MIN_DIST = 0.001  # 0.8 mm 수준 (조절 가능)

    def add_point(self, x, z):
        # self.points.append(QPointF(x, z))
        # if len(self.points) > 100:
        #     self.points.pop(0)
        # self.viewport().update()
        self.points.append(QPointF(x, z))
        # pop 필요 없음 (maxlen 자동 관리)
        self.viewport().update()

    def paintEvent(self, event):
        painter = QPainter(self.viewport())
        painter.setRenderHint(QPainter.Antialiasing)

        view_w, view_h = self.width(), self.height()
        scale_x = view_w / self.VIEW_X_RANGE
        scale_y = view_h / self.VIEW_Z_RANGE
        scale_factor = min(scale_x, scale_y) * 0.95

        transform = QTransform()
        transform.translate(view_w * 0.25, view_h * 0.95)
        transform.scale(scale_factor, -scale_factor)
        painter.setTransform(transform)

        # 축
        painter.setPen(QPen(Qt.gray, 0.005))
        painter.drawLine(QPointF(self.X_MIN, self.Z_MIN_PLOT), QPointF(self.X_MAX, self.Z_MIN_PLOT))
        painter.drawLine(QPointF(self.X_MIN, self.Z_MIN_PLOT), QPointF(self.X_MIN, self.Z_MAX_PLOT))

        # 연결 구간 조건부로만 선 그리기
        painter.setPen(QPen(QColor(50,50,255,180), 0.01))
        for i in range(len(self.points)-1):
            p1, p2 = self.points[i], self.points[i+1]
            if (p1 - p2).manhattanLength() >= self.MIN_DIST:
                painter.drawLine(p1, p2)

        # 현재 위치
        if self.points:
            painter.setBrush(Qt.red)
            painter.drawEllipse(self.points[-1], 0.015, 0.015)




# =========================================================================
# 2. 메인 RQT 위젯 클래스
# =========================================================================

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
        
        self.setWindowTitle("sky cleaner") 

        # 폰트 설정
        font = QFont()
        font.setPointSize(16)
        font.setBold(True)
        self.label_state.setFont(font)
        self.label_state.setAlignment(Qt.AlignCenter)
        self.label_state.setWordWrap(True)
        
        try:
            self.checkBox_roller.setFont(font)
            self.checkBox_cloth.setFont(font)
        except AttributeError:
            self.node.get_logger().warn("Checkboxes not found in .ui file (e.g., checkBox_roller). Skipping font change.")
            
        # Graphics View 인스턴스 대체: board_widget에 삽입
        self.graphicsView_path = CoordinateGraphicsView(self.board_widget)
        self.board_layout = QVBoxLayout(self.board_widget)
        self.board_layout.setContentsMargins(0, 0, 0, 0)
        self.board_layout.addWidget(self.graphicsView_path)
        self.board_widget.setLayout(self.board_layout)
        self.graphicsView_path.setMinimumSize(self.board_widget.sizeHint())
        self.node.get_logger().info('Custom CoordinateGraphicsView initialized and placed inside board_widget.')

        # [추가] 경로 추적 활성화 상태 변수 초기화
        self.is_path_tracking_active = False 

        # ROS2 QoS
        qos = QoSProfile(depth=1)

        # ROS2 Publishers
        self.pub_start = self.node.create_publisher(Int32, '/dsr01/task/start', qos)
        self.node.get_logger().info('ROS 2 Task control publishers initialized.')

        # progressBar 초기화
        self.progressBar.setValue(0)

        # UI 상태 저장 변수 초기화
        self.progress_value = 0
        self.status_message = "IDLE"

        # ROS2 Subscriber 설정
        self.sub_progress = self.node.create_subscription(
            Int32, '/task/progress', self.progress_callback, qos
        )
        
        self.sub_status = self.node.create_subscription(
            String, '/task/status', self.status_callback, qos
        )
        self.sub_ee_pose = self.node.create_subscription(
            Float64MultiArray, 'ee_pose_rpy', self.ee_pose_callback, qos
        )
        self.node.get_logger().info('ROS 2 Task status and EE Pose subscribers initialized.')

        # UI 상태 레이블 초기화
        self.label_state.setText("Waiting for command...")

        # 버튼 이벤트 연결
        self.push_button_start.clicked.connect(self.on_start_clicked)
        self.push_button_estop.clicked.connect(self.on_estop_clicked)

    def _format_status(self, status):
        """상태 문자열에서 접미사를 제거하고 줄바꿈 및 공백을 추가하여 가독성을 높입니다."""
        s = status
        
        if s.endswith("_COMPLETED"): s = s.replace("_COMPLETED", "")
        elif s.endswith("_STOP") or s.endswith("_PAUSED"): s = s.replace("_STOP", "").replace("_PAUSED", "")
        elif s.endswith("_RUNNING"): s = s.replace("_RUNNING", "")
        elif s.endswith("_ERROR"): s = s.replace("_ERROR", "")
        
        s = s.replace("FULL_TASK_ROLLER_PHASE", "FULL TASK\n(ROLLER PHASE)")
        s = s.replace("FULL_TASK_CLOTH_PHASE", "FULL TASK\n(CLOTH PHASE)")
        s = s.replace("ROLLER_TASK", "ROLLER TASK")
        s = s.replace("CLOTH_TASK", "CLOTH TASK")
        s = s.replace("_", " ")
        
        return s

    def update_ui_label(self):
        """진행도와 상태 메시지를 통합하여 UI 라벨을 업데이트"""
        progress = self.progress_value
        status = self.status_message
        task_info = self._format_status(status)
        
        if status == "IDLE":
            display_text = "Waiting for command..."
        elif status.endswith("_COMPLETED"):
            display_text = f"✅ COMPLETED:\n{task_info} (100%)"
        elif status.endswith("_STOP") or status.endswith("_PAUSED"):
            display_text = f"⏸ PAUSED:\n{task_info} ({progress}%)"
        elif status.endswith("_ERROR"):
            display_text = f"❌ ERROR:\n{task_info} ({progress}%)"
        else: # RUNNING 상태
            display_text = f"▶ RUNNING:\n{task_info} ({progress}%)"
            
        self.label_state.setText(display_text)
        self.progressBar.setValue(progress)

    def on_start_clicked(self):
        # START 버튼 클릭 시, 기존 경로를 초기화합니다.
        self.graphicsView_path.points.clear()
        self.graphicsView_path.viewport().update()
        
        roller_checked = self.checkBox_roller.isChecked()
        cloth_checked = self.checkBox_cloth.isChecked()

        if roller_checked and cloth_checked: msg_value = 3
        elif roller_checked: msg_value = 1
        elif cloth_checked: msg_value = 2
        else: msg_value = 5

        self.pub_start.publish(Int32(data=msg_value))
        self.node.get_logger().info(f"START clicked: Int32={msg_value}")


    def on_estop_clicked(self):
        self.pub_start.publish(Int32(data=0))
        self.node.get_logger().info('EMERGENCY STOP clicked: Sent CMD 0')

    def progress_callback(self, msg: Int32):
        """Int형 메시지를 받아 진행도 값을 저장하고 UI 업데이트 요청"""
        self.progress_value = max(0, min(100, msg.data))
        self.update_ui_label()

    def status_callback(self, msg: String):
        """
        [수정] 상태에 따라 경로 추적 플래그를 업데이트합니다.
        """
        self.status_message = msg.data
        current_status = self.status_message
        
        # RUNNING 상태이고 'ROLLER' 또는 'CLOTH' 키워드가 포함되어 있으면 활성화
        if "_RUNNING" in current_status:
            if "ROLLER" in current_status or "CLOTH" in current_status:
                self.is_path_tracking_active = True
            else:
                self.is_path_tracking_active = False
        else:
            self.is_path_tracking_active = False
            
        self.update_ui_label()
        
    def ee_pose_callback(self, msg: Float64MultiArray):
        """
        [핵심 수정] 경로 추적 활성화 상태이면서 Z좌표가 유효한 경우에만 좌표를 추출하여 그립니다.
        """
        if hasattr(self, 'is_path_tracking_active') and self.is_path_tracking_active and len(msg.data) >= 3:
            
            x_coord = msg.data[0]
            z_coord = msg.data[2]
            
            # 1. X축은 1사분면(X>=0) 검사
            # 2. Z축은 유효한 작업 높이(Z_REAL_MIN) 검사
            if z_coord >= self.graphicsView_path.Z_MIN_PLOT:
                self.graphicsView_path.add_point(x_coord, z_coord)