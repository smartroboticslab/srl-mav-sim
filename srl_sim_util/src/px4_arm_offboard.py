#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause
import mavros_msgs.srv
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


def loginfo(s):
    rospy.loginfo(rospy.get_name() + ": " + s)


if __name__ == '__main__':
    rospy.init_node('px4_arm_offboard')
    loginfo('Waiting for MAVROS services')
    rospy.wait_for_service('/mavros/cmd/arming')
    arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    rospy.wait_for_service('/mavros/set_mode')
    mode_srv = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

    rate = rospy.Rate(10)
    rospy.sleep(10)
    msg = rospy.wait_for_message('/okvis_node/okvis_odometry', Odometry)

    while not arm_srv(True).success:
        rate.sleep()
    loginfo('MAV armed')

    while not mode_srv(0, 'OFFBOARD').mode_sent:
        rate.sleep()
    loginfo('MAV switched to OFFBOARD mode')
