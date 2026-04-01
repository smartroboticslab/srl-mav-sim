# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0
cmake_minimum_required(VERSION 3.18)
project(srl_sim_gz VERSION 0.1.0)

# The directory where the PX4-Autopilot submodule is located.
set(PX4_DIR "${CMAKE_SOURCE_DIR}/../PX4-Autopilot")

# Run the command DONT_RUN=1 make px4_sitl_default in the PX4-Autopilot directory
# to build the PX4 SITL target. This is necessary because the PX4-Autopilot
# submodule is not a catkin package and cannot be built using the standard
# catkin build process.
add_custom_command(
	OUTPUT "${PX4_DIR}/build/px4_sitl_default"
	COMMAND ${CMAKE_COMMAND} -E chdir "${PX4_DIR}" make px4_sitl_default
	WORKING_DIRECTORY "${PX4_DIR}"
	COMMENT "Building PX4 SITL target"
)
add_custom_target(build_px4_sitl ALL DEPENDS "${PX4_DIR}/build/px4_sitl_default")

# Build the px4-gzsim-plugins
add_custom_target(build_px4_gzsim_plugins ALL
	COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_CURRENT_SOURCE_DIR}/px4-gzsim-plugins bash build.sh
	COMMENT "Running build.sh in the px4-gzsim-plugins directory"
)


find_package(ament_cmake REQUIRED)

# install launch scripts for sitl and gazebo
install(
	PROGRAMS src/px4_sitl.sh src/gazebo.bash
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${PX4_DIR}/build/px4_sitl_default/bin 
	DESTINATION 
		share/${PROJECT_NAME}/
	FILE_PERMISSIONS
		OWNER_READ OWNER_WRITE OWNER_EXECUTE
		GROUP_READ GROUP_EXECUTE
		WORLD_READ WORLD_EXECUTE)
install(DIRECTORY ${PX4_DIR}/build/px4_sitl_default/etc 
	DESTINATION 
		share/${PROJECT_NAME}/
	FILE_PERMISSIONS
		OWNER_READ OWNER_WRITE OWNER_EXECUTE
		GROUP_READ GROUP_EXECUTE
		WORLD_READ WORLD_EXECUTE)


# install gzsim-plugins shared library
install(FILES
	${CMAKE_CURRENT_SOURCE_DIR}/px4-gzsim-plugins/build/libmavlink_hitl_gazebosim.so
	DESTINATION share/${PROJECT_NAME}
)

# install docker launch script
install(FILES
	${CMAKE_CURRENT_SOURCE_DIR}/../docker/launch_docker.sh
	DESTINATION share/${PROJECT_NAME}
	PERMISSIONS
		OWNER_READ OWNER_WRITE OWNER_EXECUTE
		GROUP_READ GROUP_EXECUTE
		WORLD_READ WORLD_EXECUTE
)

# install launch files and resources
install(DIRECTORY 
	launch
	resources
	config
	DESTINATION share/${PROJECT_NAME}/)
ament_package()

