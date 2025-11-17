import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 # Int32 메시지 타입을 임포트

class SimplePublisher(Node):

    def __init__(self):
        # 노드 이름 설정
        super().__init__('progress_publisher_node')
        
        # 1. 퍼블리셔 생성: Int32 메시지, 토픽 이름 'task/progress', QoS 깊이 10
        self.publisher_ = self.create_publisher(Int32, 'task/progress', 10)
        
        # 2. 전송 주기 설정: 2.0초 (500ms * 4 = 2000ms는 2.0초와 동일)
        timer_period = 2.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 3. 카운터 변수 초기화
        self.i = 0

    def timer_callback(self):
        # Int32 메시지 객체 생성
        msg = Int32()
        
        # 카운터 값을 메시지 데이터에 저장
        msg.data = self.i
        
        # 메시지 전송 (퍼블리시)
        self.publisher_.publish(msg)
        
        # 콘솔에 전송 정보 출력
        self.get_logger().info(f'Publishing to task/progress: "{msg.data}"')
        
        # 카운터 증가
        self.i += 1

def main(args=None):
    # ROS 2 초기화
    rclpy.init(args=args)

    # 노드 인스턴스 생성
    simple_publisher = SimplePublisher()

    # 노드 실행 (타이머 콜백이 주기적으로 실행됨)
    rclpy.spin(simple_publisher)

    # Ctrl+C 등으로 종료 시
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()