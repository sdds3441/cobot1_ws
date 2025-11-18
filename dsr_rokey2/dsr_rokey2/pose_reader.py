#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def quat_to_rpy(qx, qy, qz, qw):
    # roll (x)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class EEOnceFromTF(Node):
    def __init__(self,
                 base_frame: str = 'base_link',
                 ee_frame: str = 'link_6'):
        super().__init__('ee_once_from_tf')

        self.base_frame = base_frame
        self.ee_frame = ee_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TF 들어올 시간 약간 기다렸다가 한 번만 읽고 종료
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.tried = 0

    def timer_callback(self):
        self.tried += 1
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                Time()  # latest
            )

            t = trans.transform.translation
            q = trans.transform.rotation

            x, y, z = t.x, t.y, t.z
            roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)

            print(f"At time {trans.header.stamp.sec}.{trans.header.stamp.nanosec}")
            print(f"- Translation: [{x:.3f}, {y:.3f}, {z:.3f}]")
            print(f"- Rotation: in Quaternion (xyzw) "
                  f"[{q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}]")
            print(f"- Rotation: in RPY (radian) "
                  f"[{roll:.3f}, {pitch:.3f}, {yaw:.3f}]")
            print(f"- Rotation: in RPY (degree) "
                  f"[{math.degrees(roll):.3f}, "
                  f"{math.degrees(pitch):.3f}, "
                  f"{math.degrees(yaw):.3f}]")

            # 한 번 찍고 종료
            rclpy.shutdown()

        except Exception as e:
            if self.tried > 10:
                self.get_logger().error(f"TF lookup 실패: {e}")
                rclpy.shutdown()
            else:
                self.get_logger().warn("TF 기다리는 중...")


def main():
    rclpy.init()
    node = EEOnceFromTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
