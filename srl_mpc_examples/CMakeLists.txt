# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0
cmake_minimum_required(VERSION 3.8)
project(srl_mpc_examples VERSION 0.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_interface_msgs REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
	example_mpc_path.py
	gt_adapter_gazebo_classic.py
	scripts/publish_trajectory.py
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	data
	config
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
