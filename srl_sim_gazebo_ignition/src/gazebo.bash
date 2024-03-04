#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2023-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023-2024 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0
set -e

# roslaunch will pass extra arguments so we must accept more than 2.
if [ "$#" -lt 2 ]
then
	printf 'Usage: %s PX4_DIR WORLD\n' "${0##*/}"
	exit 2
fi

px4_dir="$1"
build_dir="$px4_dir/build/px4_sitl_default"

export IGN_GAZEBO_RESOURCE_PATH=$px4_dir/../srl_sim_gazebo_ignition/resources:$IGN_GAZEBO_RESOURCE_PATH
source "$px4_dir/Tools/setup_ignition.bash" "$px4_dir" "$build_dir"
ign gazebo --force-version 6 ${HEADLESS+-s} -r "$2"
