#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause
import math
import rospy
from mav_interface_msgs.msg import FullStateStamped, FullStateTrajectory, Waypoint

if __name__ == '__main__':
    rospy.init_node('mpc_trajectory')

    tsv_file = rospy.get_param('~tsv_file')
    flush_queue = rospy.get_param('~flush_queue', False)
    read_every_n = rospy.get_param('~read_every_n', 1)
    if read_every_n < 1:
        rospy.logfatal('Expected positive integer for parameter read_every_n, not {}'.format(read_every_n))
        raise ValueError

    pub = rospy.Publisher('~trajectory', FullStateTrajectory, queue_size=10)

    # Read the trajectory
    msg = FullStateTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.taskID = 'trajectory'
    msg.flushReferenceQueue = flush_queue
    with open(tsv_file) as f:
        # Skip the TSV header
        next(f)
        for i, line in enumerate(f):
            if i % read_every_n != 0:
                continue
            columns = line.rstrip(' \r\n').split('\t')
            if i == 0:
                t_init = int(columns[0])
            state = FullStateStamped()
            # Timestamps must be relative to the first trajectory state
            state.timestampNanoSeconds = int(columns[0]) - t_init
            state.position.x = float(columns[1])
            state.position.y = float(columns[2])
            state.position.z = float(columns[3])
            state.orientation.x = float(columns[4])
            state.orientation.y = float(columns[5])
            state.orientation.z = float(columns[6])
            state.orientation.w = float(columns[7])
            msg.trajectory.append(state)
    # Move to the first trajectory state before executing the trajectory
    first_state = msg.trajectory[0]
    msg.initialWaypoint = Waypoint(position=first_state.position,
                                   orientation=first_state.orientation,
                                   positionTolerance=0.2,
                                   orientationTolerance=math.radians(20))

    # Wait for a subscriber before publishing
    while pub.get_num_connections() == 0:
        rospy.sleep(0.5)
    pub.publish(msg)
    rospy.loginfo('Published trajectory')
