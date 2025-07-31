#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause
import rclpy
import mavros_msgs.srv
import threading
from rclpy.executors import ExternalShutdownException
 
def loginfo(s):
    # Use ROS 2 logging
    rclpy.logging.get_logger("px4_arm_offboard").info(f"{s}")
 
def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
 
if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('px4_arm_offboard')
 
    t = threading.Thread(target=spin_in_background)
    t.start()
    
    rclpy.get_global_executor().add_node(node)
 
    loginfo('Waiting for MAVROS services')
    arm_client = node.create_client(mavros_msgs.srv.CommandBool, '/mavros/cmd/arming')
    while not arm_client.wait_for_service(timeout_sec=1.0):
        loginfo('Waiting for /mavros/cmd/arming service...')
    mode_client = node.create_client(mavros_msgs.srv.SetMode, '/mavros/set_mode')
    while not mode_client.wait_for_service(timeout_sec=1.0):
        loginfo('Waiting for /mavros/set_mode service...')
    
    rate = node.create_rate(10)
    loginfo('all services available')
 
    while rclpy.ok():
        arm_req = mavros_msgs.srv.CommandBool.Request(value=True)
        arm_future = arm_client.call_async(arm_req)
        rclpy.spin_until_future_complete(node, arm_future)
        if arm_future.result() is not None and arm_future.result().success:
            break
        rate.sleep()
    loginfo('MAV armed')
 
    while rclpy.ok():
        mode_req = mavros_msgs.srv.SetMode.Request(custom_mode='OFFBOARD')
        mode_future = mode_client.call_async(mode_req)
        rclpy.spin_until_future_complete(node, mode_future)
        if mode_future.result() is not None and mode_future.result().mode_sent:
            break
        rate.sleep()
    loginfo('MAV switched to OFFBOARD mode')
 
    t.join()