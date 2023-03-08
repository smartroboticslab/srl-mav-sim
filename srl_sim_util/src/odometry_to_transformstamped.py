#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause
import rospy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdometryToTransformStamped:
    """
    Publish a nav_msgs/Odometry message as a geometry_msgs/TransformStamped
    message.
    """
    def __init__(self):
        rospy.init_node("odometry_to_transformstamped")
        rospy.Subscriber("~odometry", Odometry, self.callback, queue_size=20)
        self._pub = rospy.Publisher("~transform", TransformStamped, queue_size=20)
        rospy.spin()

    def callback(self, msg: Odometry):
        msg_out = TransformStamped()
        msg_out.header = msg.header
        msg_out.child_frame_id = msg.child_frame_id
        msg_out.transform.translation.x = msg.pose.pose.position.x
        msg_out.transform.translation.y = msg.pose.pose.position.y
        msg_out.transform.translation.z = msg.pose.pose.position.z
        msg_out.transform.rotation = msg.pose.pose.orientation
        self._pub.publish(msg_out)


if __name__ == "__main__":
    try:
        node = OdometryToTransformStamped()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
