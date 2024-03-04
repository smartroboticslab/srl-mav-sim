#!/bin/sh
# SPDX-FileCopyrightText: 2023-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023-2024 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0
set -eu

# roslaunch will pass extra arguments so we must accept more than 2.
if [ "$#" -lt 2 ]
then
	printf 'Usage: %s PX4_DIR AIRFRAME\n' "${0##*/}"
	exit 2
fi

build_dir="$1/build/px4_sitl_default"

env PX4_SIM_MODEL="$2" "$build_dir/bin/px4" -d -s etc/init.d-posix/rcS "$build_dir/etc"
