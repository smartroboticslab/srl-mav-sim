#!/usr/bin/env python3
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("test_offboard_mode")
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    rospy.loginfo("Connected!")

    # Send a few setpoints before starting
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2
    for i in range(100):
        if(rospy.is_shutdown()):
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    # Offboard mode
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    set_mode_client.call(offb_set_mode)
    rospy.loginfo("Sent offboard mode!")
    time.sleep(5)

    # Arm Command
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    arming_client.call(arm_cmd)
    rospy.loginfo("Sent ARM command!")
    rate.sleep()

    # Keep publishing pose setpoint
    rospy.loginfo("Sending position setpoint!")
    while(not rospy.is_shutdown()):
        local_pos_pub.publish(pose)
        rate.sleep()
