import rclpy
from rclpy.node import Node
from dsr_interfaces.msg import Cleanliness  # master_runner가 발행하는 메시지

class StatusMonitor(Node):
    """
    'cleaning_status' 토픽을 구독하여,
    수신된 청결도와 진행률 메시지를 터미널에 출력하는 노드입니다.
    """
    def __init__(self):
        super().__init__('status_monitor_node')
        
        # 네임스페이스를 포함한 전체 토픽 이름으로 수정합니다.
        self.topic_name = '/dsr01/cleaning_status'
        
        self.subscription = self.create_subscription(
            Cleanliness,
            self.topic_name,
            self.listener_callback,  # 메시지가 도착하면 이 함수를 실행
            10)
        
        self.get_logger().info(f"'{self.topic_name}' 토픽 구독을 시작합니다. 메시지 대기 중...")

    def listener_callback(self, msg):
        """
        메시지를 수신했을 때마다 호출되는 콜백 함수
        """
        # 수신된 메시지의 내용을 터미널에 INFO 레벨로 출력합니다.
        self.get_logger().info(
            f"수신: [작업: {msg.task_name}] "
            f"[진행률: {msg.progress_percentage:.1f}%] "
            f"[청결도: {msg.cleanliness_level:.1f}%]"
        )

        # 진행률이 100%에 도달하면 노드를 자동으로 종료합니다.
        if msg.progress_percentage >= 100.0:
            self.get_logger().info("전체 작업 100% 완료 메시지 수신. 모니터 노드를 종료합니다.")
            self.destroy_node() # 노드 종료 요청

def main(args=None):
    rclpy.init(args=args)

    status_monitor = StatusMonitor()

    try:
        # 노드를 계속 실행하며 메시지를 기다립니다.
        rclpy.spin(status_monitor)
    except KeyboardInterrupt:
        print("모니터 노드 종료...")
    finally:
        # 노드 종료
        status_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()