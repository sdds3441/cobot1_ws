# pose_logger.py
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import GetCurrentPosx

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')

        self.cli = self.create_client(
            GetCurrentPosx, '/dsr01/aux_control/get_current_posx'
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")

        self.get_logger().info("Connected to get_current_posx")

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        req = GetCurrentPosx.Request()
        req.ref = 0

        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return

        if not res.success:
            return

        arr = res.task_pos_info[0].data
        x, y, z, rx, ry, rz = arr[:6]
        self.get_logger().info(
            f"POSE xyz: {[round(v,3) for v in [x,y,z]]}, "
            f"rpy: {[round(v,3) for v in [rx,ry,rz]]}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
