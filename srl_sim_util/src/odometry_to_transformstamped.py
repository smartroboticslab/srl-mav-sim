#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdometryToTransformStamped(Node):
    """
    Publish a nav_msgs/Odometry message as a geometry_msgs/TransformStamped
    message.
    """
    def __init__(self):
        super().__init__("odometry_to_transformstamped")
        self.create_subscription(
            Odometry,
            "odometry",
            self.callback,
            queue_size=20)
        self._pub = self.create_publisher(
            TransformStamped,
            "transform",
            queue_size=20)

    def callback(self, msg: Odometry):
        msg_out = TransformStamped()
        msg_out.header = msg.header
        msg_out.child_frame_id = msg.child_frame_id
        msg_out.transform.translation.x = msg.pose.pose.position.x
        msg_out.transform.translation.y = msg.pose.pose.position.y
        msg_out.transform.translation.z = msg.pose.pose.position.z
        msg_out.transform.rotation = msg.pose.pose.orientation
        self._pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryToTransformStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
