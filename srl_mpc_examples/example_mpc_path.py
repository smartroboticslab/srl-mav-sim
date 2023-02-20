#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause
import rospy

from math import cos, sin, pi, tau
from geometry_msgs.msg import Vector3, Quaternion
from mav_interface_msgs.msg import Path, Waypoint
from typing import List


def circular_path(center: Vector3, radius: float, num_points: int) -> Path:
    tt = [tau / num_points * x for x in range(num_points)]
    pos_x = [radius * cos(t) + center.x for t in tt]
    pos_y = [radius * sin(t) + center.y for t in tt]
    positions = [Vector3(x, y, center.z) for x, y in zip(pos_x, pos_y)]
    orientations = [Quaternion(z=sin((t + pi) / 2), w=cos((t + pi) / 2)) for t in tt]
    waypoints=[Waypoint(
            position=p,
            orientation=o,
            positionTolerance=0.1,
            orientationTolerance=0.15
        ) for p, o in zip(positions, orientations)],
    return Path(taskID="circle", waypoints=waypoints, flushReferenceQueue=False)


if __name__ == "__main__":
    try:
        # TODO integrate switch to offboard mode, take off, etc.
        rospy.init_node("example_mpc_path")
        path = circular_path(Vector3(0, 0, 2), 2, 60)
        output_topic = "~path"
        pub = rospy.Publisher(output_topic, Path, queue_size=2)
        rate = rospy.Rate(0.05)
        while not rospy.is_shutdown():
            pub.publish(path)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
