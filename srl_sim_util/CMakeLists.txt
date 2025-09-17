# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0
cmake_minimum_required(VERSION 3.8..3.30)
project(srl_sim_util VERSION 0.1.0)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

# Install Python scripts
install(
	PROGRAMS 
		src/odometry_to_transformstamped.py
		src/px4_arm_offboard.py
	DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
	DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}/
)

ament_package()
