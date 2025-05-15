#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
import mavros_msgs.srv
from nav_msgs.msg import Odometry


def loginfo(s):
    # Use ROS 2 logging
    rclpy.logging.get_logger("px4_arm_offboard").info(f"{s}")

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('px4_arm_offboard')

    loginfo('Waiting for MAVROS services')
    arm_client = node.create_client(mavros_msgs.srv.CommandBool, '/mavros/cmd/arming')
    while not arm_client.wait_for_service(timeout_sec=1.0):
        loginfo('Waiting for /mavros/cmd/arming service...')
    mode_client = node.create_client(mavros_msgs.srv.SetMode, '/mavros/set_mode')
    while not mode_client.wait_for_service(timeout_sec=1.0):
        loginfo('Waiting for /mavros/set_mode service...')
    
    rate = node.create_rate(10)
    loginfo('all services available')

    while not arm_client.call(mavros_msgs.srv.CommandBool.Request(value=True)).success:
        rate.sleep()
    loginfo('MAV armed')

    while not mode_client.call(mavros_msgs.srv.SetMode.Request(custom_mode='OFFBOARD')).mode_sent:
        rate.sleep()
    loginfo('MAV switched to OFFBOARD mode')
