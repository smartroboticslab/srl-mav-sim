#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause
import rospy

from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import TransformStamped
from typing import List, Optional


class GazeboGtAdapter:
    def __init__(self):
        name = "gazebo_classic_gt_adapter"
        rospy.init_node(name)
        vehicle = rospy.get_param("~" + name + "/vehicle", "dji_f450_srl")
        input_topic = "/gazebo/link_states"
        output_topic = "/gazebo/" + vehicle + "/pose"
        self._vehicle_base_link = vehicle + "::base_link"
        rospy.Subscriber(input_topic, LinkStates, self._callback, queue_size=20)
        self._pub = rospy.Publisher(output_topic, TransformStamped, queue_size=20)
        rospy.loginfo("{}: {} --> {}".format(name, input_topic, output_topic))
        rospy.spin()

    def _link_index(self, names: List[str]) -> Optional[int]:
        for i, n in enumerate(names):
            if n == self._vehicle_base_link:
                return i
        return None

    def _callback(self, msg: LinkStates):
        # Gazebo doesn't include a timestamp in the LinkStates message, use the
        # current time instead.
        t = rospy.get_rostime()
        idx = self._link_index(msg.name)
        msg_out = TransformStamped()
        msg_out.header.stamp = t
        msg_out.header.frame_id = "map"
        msg_out.child_frame_id = "body"
        msg_out.transform.translation = msg.pose[idx].position
        msg_out.transform.rotation = msg.pose[idx].orientation
        self._pub.publish(msg_out)


if __name__ == "__main__":
    try:
        node = GazeboGtAdapter()
    except rospy.ROSInterruptException:
        pass
