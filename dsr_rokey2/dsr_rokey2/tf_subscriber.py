#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class EEPoseRPYSubscriber(Node):
    def __init__(self):
        super().__init__('ee_pose_rpy_subscriber')

        self.sub = self.create_subscription(
            Float64MultiArray,
            'ee_pose_rpy',
            self.callback,
            10
        )

        self.get_logger().info(">>> ee_pose_rpy_subscriber started (subscribe /ee_pose_rpy)")

    def callback(self, msg: Float64MultiArray):
        # data: [x, y, z, roll_deg, pitch_deg, yaw_deg]
        if len(msg.data) < 6:
            self.get_logger().warn("msg.data 길이가 6보다 작음")
            return

        x, y, z, roll_deg, pitch_deg, yaw_deg = msg.data

        print(f"- Translation: [{x:.3f}, {y:.3f}, {z:.3f}]")
        print(f"- Rotation: in RPY (deg) "
              f"[{roll_deg:.2f}, {pitch_deg:.2f}, {yaw_deg:.2f}]")
        print("=====")


def main():
    rclpy.init()
    node = EEPoseRPYSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
