# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0
cmake_minimum_required(VERSION 3.10)
project(srl_mav_sim_gazebo_classic VERSION 0.0.0)

find_package(catkin REQUIRED)

# The directory where the PX4-Autopilot submodule is located.
set(PX4_DIR "${CMAKE_SOURCE_DIR}/../PX4-Autopilot")
# Use a custom target to build PX4 since building it through catkin won't work
# as intended. This will probably not work for PX4-Autopilot versions later
# than v1.13.2.
add_custom_target(px4 ALL
	COMMAND cmake -E chdir "${PX4_DIR}" env DONT_RUN=1 make px4_sitl_default gazebo)
