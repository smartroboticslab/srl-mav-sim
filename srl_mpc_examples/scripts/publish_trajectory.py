#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause
import math
import rclpy
from rclpy.node import Node
from mav_interface_msgs.msg import FullStateStamped, FullStateTrajectory, Waypoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('mpc_trajectory')
        
        # Declare parameters
        self.declare_parameter('tsv_file', '')
        self.declare_parameter('flush_queue', False)
        self.declare_parameter('read_every_n', 1)
        
        # Get parameters
        tsv_file = self.get_parameter('tsv_file').get_parameter_value().string_value
        flush_queue = self.get_parameter('flush_queue').get_parameter_value().bool_value
        read_every_n = self.get_parameter('read_every_n').get_parameter_value().integer_value
        
        if read_every_n < 1:
            self.get_logger().fatal(f'Expected positive integer for parameter read_every_n, not {read_every_n}')
            raise ValueError
        
        self.pub = self.create_publisher(FullStateTrajectory, 'trajectory', 10)
        
        # Read the trajectory
        msg = FullStateTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = 'trajectory'
        msg.flush_reference_queue = flush_queue
        
        with open(tsv_file) as f:
            # Skip the TSV header
            next(f)
            t_init = None
            for i, line in enumerate(f):
                if i % read_every_n != 0:
                    continue
                columns = line.rstrip(' \r\n').split('\t')
                if t_init is None:
                    t_init = int(columns[0])
                state = FullStateStamped()
                # Timestamps must be relative to the first trajectory state
                state.timestamp_nano_seconds = int(columns[0]) - t_init
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
        msg.initial_waypoint = Waypoint(position=first_state.position,
                                      orientation=first_state.orientation,
                                      position_tolerance=0.2,
                                      orientation_tolerance=math.radians(20))
        
        # Wait for a subscriber before publishing
        while self.pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.5)
        
        self.pub.publish(msg)
        self.get_logger().info('Published trajectory')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
