# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
# SPDX-FileCopyrightText: 2024 Sebastián Barbas Laina
# SPDX-License-Identifier: CC0-1.0
cmake_minimum_required(VERSION 3.10)
project(srl_sim_gazebo_ignition VERSION 1.0.0)
find_package(ament_cmake REQUIRED)

# The directory where the PX4-Autopilot submodule is located.
set(PX4_DIR "${CMAKE_SOURCE_DIR}/../PX4-Autopilot")
# Use a custom target to build PX4 since building it through catkin won't work
# as intended. This will probably not work for PX4-Autopilot versions later
# than v1.13.2.
#add_custom_target(px4 ALL COMMAND cmake -E chdir "${PX4_DIR}" env DONT_RUN=1 make px4_sitl gz_x500)


install(
	PROGRAMS src/px4_sitl.sh src/gazebo.bash
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
