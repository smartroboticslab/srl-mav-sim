# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0
cmake_minimum_required(VERSION 3.10)
project(srl_mav_sim_gazebo_classic VERSION 0.0.0)

find_package(catkin REQUIRED)

# Build PX4-Autopilot with custom airframe ####################################
# The filenames of the custom airframes. Each filename must consist of one or
# more sigits, followed by an underscore, followed by the name of the airframe.
set(AIRFRAMES 4021_dji_f450_srl 4022_dji_s550_srl)
# The directory where the PX4-Autopilot submodule is located.
set(PX4_DIR "${CMAKE_SOURCE_DIR}/../PX4-Autopilot")
# The directory where PX4 airframe config files are located.
set(AIRFRAME_DIR "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes")
foreach(AIRFRAME ${AIRFRAMES})
	# Copy the custom MAV airframe to PX4-Autopilot. Changing airframes
	# requires a clean build of PX4-Autopilot so run `make clean`. This
	# custom command will ensure `make clean` only runs when the airframe
	# has been modified.
	add_custom_command(OUTPUT "${AIRFRAME_DIR}/${AIRFRAME}"
		COMMAND cmake -E chdir "${PX4_DIR}" make clean
		COMMAND cmake -E copy "${CMAKE_SOURCE_DIR}/airframes/${AIRFRAME}" "${AIRFRAME_DIR}"
		MAIN_DEPENDENCY "${CMAKE_SOURCE_DIR}/airframes/${AIRFRAME}")
	# Update the CMakeLists.txt to use the added airframe if it doesn't already
	# contain it using an awk script. Escape the backslashes in the awk script
	# since backslash is an escape charecter in CMake strings. Write the modified
	# CMakeLists.txt to a temporary file and then move it over the original since
	# redirecting standard output to the file used for input isn't safe.
	execute_process(
		COMMAND awk "/^[[:space:]]*${AIRFRAME}[[:space:]]*$/ { found = 1 } /^[[:space:]]*)[[:space:]]*$/ { if (!found) printf \"\\t${AIRFRAME}\\n\" } { print }" "${AIRFRAME_DIR}/CMakeLists.txt"
		OUTPUT_FILE "${AIRFRAME_DIR}/CMakeLists.txt.new")
	file(RENAME "${AIRFRAME_DIR}/CMakeLists.txt.new" "${AIRFRAME_DIR}/CMakeLists.txt")
	# Collect all custom airframe files to use as dependencies to the px4
	# target.
	set(PX4_DEPENDENCIES "${PX4_DEPENDENCIES}" "${AIRFRAME_DIR}/${AIRFRAME}")
endforeach()
# Use a custom target to build PX4 since building it through catkin won't work
# as intended. This will probably not work for PX4-Autopilot versions later
# than v.1.13.2.
add_custom_target(px4 ALL
	COMMAND cmake -E chdir "${PX4_DIR}" env DONT_RUN=1 make px4_sitl_default gazebo
	DEPENDS "${PX4_DEPENDENCIES}")
