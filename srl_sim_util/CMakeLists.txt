# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0
cmake_minimum_required(VERSION 3.0.2)
project(srl_sim_util VERSION 0.1.0)

find_package(catkin REQUIRED
	COMPONENTS
		roscpp
		rospy
)

catkin_package()

catkin_install_python(
	PROGRAMS src/odometry_to_transformstamped.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
