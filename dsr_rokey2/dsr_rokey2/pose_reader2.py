import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NumberStringPublisher(Node):
    def __init__(self):
        super().__init__('number_string_publisher')
        
        # publisher 생성
        self.publisher = self.create_publisher(String, '/dsr01/task_command', 10)

        # 1부터 시작
        self.counter = 1

        # 0.5초마다 timer 콜백 실행
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("Number String Publisher Started")

    def timer_callback(self):
        msg = String()
        msg.data = str(self.counter)   # 숫자를 문자열로 변환
        self.publisher.publish(msg)

        self.get_logger().info(f"Published: '{msg.data}'")

        self.counter += 1  # 다음 숫자를 위해 증가


def main(args=None):
    rclpy.init(args=args)
    node = NumberStringPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
