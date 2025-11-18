#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dsr_msgs2.msg import RobotState

TOPIC = "/dsr01/robot_state"

class TcpReader(Node):
    def __init__(self):
        super().__init__("tcp_reader")

        self.create_subscription(
            RobotState,
            TOPIC,
            self.cb,
            10
        )

        self.get_logger().info("Reading actual_tcp_position from robot controller")

    def cb(self, msg: RobotState):
        tcp = msg.actual_tcp_position  # [x, y, z, a, b, c]
        vel = msg.actual_tcp_velocity  # [vx, vy, vz, wx, wy, wz]

        x, y, z = tcp[0], tcp[1], tcp[2]
        a, b, c = tcp[3], tcp[4], tcp[5]

        self.get_logger().info(
            f"TCP pos: ({x:.3f}, {y:.3f}, {z:.3f}) m, "
            f"rpy: ({a:.3f}, {b:.3f}, {c:.3f}) rad"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TcpReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
