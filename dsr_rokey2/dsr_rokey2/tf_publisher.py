#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray   # ✅ 추가


def quat_to_rpy(qx, qy, qz, qw):
    """쿼터니언 -> RPY(rad)"""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class EEPoseTF(Node):
    def __init__(self,
                 base_frame: str = 'base_link',
                 ee_frame: str = 'link_6'):
        super().__init__('ee_pose_from_tf')

        self.base_frame = base_frame
        self.ee_frame = ee_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # PoseStamped 퍼블리셔 (원래 있던 거)
        self.pose_pub = self.create_publisher(PoseStamped, 'ee_pose', 10)

        # ✅ 계산된 값(x, y, z, roll_deg, pitch_deg, yaw_deg)을 보내는 퍼블리셔
        self.pose_rpy_pub = self.create_publisher(
            Float64MultiArray,
            'ee_pose_rpy',
            10
        )

        # 0.1초마다 TF 읽기
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(">>> ee_pose_from_tf node started (publish /ee_pose, /ee_pose_rpy)")

    def timer_callback(self):
        try:
            # base_frame 기준 ee_frame의 TF 조회
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                Time()
            )

            t = trans.transform.translation
            q = trans.transform.rotation

            x, y, z = t.x, t.y, t.z
            roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)

            # 각도(rad → deg)
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)

            # PoseStamped 메세지 생성 & 퍼블리시 (기존 용도)
            msg_pose = PoseStamped()
            msg_pose.header.stamp = self.get_clock().now().to_msg()
            msg_pose.header.frame_id = self.base_frame
            msg_pose.pose.position.x = x
            msg_pose.pose.position.y = y
            msg_pose.pose.position.z = z
            msg_pose.pose.orientation.x = q.x
            msg_pose.pose.orientation.y = q.y
            msg_pose.pose.orientation.z = q.z
            msg_pose.pose.orientation.w = q.w
            self.pose_pub.publish(msg_pose)

            # ✅ 배열 형태로 x,y,z, roll_deg, pitch_deg, yaw_deg 보내기
            msg_rpy = Float64MultiArray()
            msg_rpy.data = [x, y, z, roll_deg, pitch_deg, yaw_deg]
            self.pose_rpy_pub.publish(msg_rpy)

            # 콘솔 로그 (디버깅 용)
            # print(f"- Translation: [{x:.3f}, {y:.3f}, {z:.3f}]")
            # print(f"- Rotation: in RPY (degree) "
            #       f"[{roll_deg:.2f}, {pitch_deg:.2f}, {yaw_deg:.2f}]")
            # print("-----")
            print("published")
        except Exception as e:
            print(f"TF lookup 실패: {e}")


def main():
    rclpy.init()
    node = EEPoseTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
