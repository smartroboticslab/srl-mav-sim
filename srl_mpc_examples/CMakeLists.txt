# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0
cmake_minimum_required(VERSION 3.10)
project(srl_mpc_examples VERSION 0.0.0)

find_package(catkin REQUIRED COMPONENTS
	gazebo_msgs
	geometry_msgs
	mav_interface_msgs
	rospy
)

catkin_package(
	CATKIN_DEPENDS
		gazebo_msgs
		geometry_msgs
		mav_interface_msgs
		message_runtime
		rospy
)

catkin_install_python(
	PROGRAMS gt_adapter_gazebo_classic.py example_mpc_path.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
