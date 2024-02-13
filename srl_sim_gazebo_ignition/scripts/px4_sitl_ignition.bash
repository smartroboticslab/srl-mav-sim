#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2023-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023-2024 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0
px4_dir="$1"
world="$2"
airframe="$3"
build_dir="$px4_dir/build/px4_sitl_default"

# Kill any running instances of Ignition Gazebo now and once the script exits.
pkill -x -KILL '^ign server'
trap 'pkill -x -KILL "^ign server" || true' EXIT INT QUIT ABRT TERM

# Launch Ignition Gazebo.
export IGN_GAZEBO_RESOURCE_PATH=$px4_dir/../srl_sim_gazebo_ignition/resources:$IGN_GAZEBO_RESOURCE_PATH
source "$px4_dir/Tools/setup_ignition.bash" "$px4_dir" "$build_dir"
ign gazebo --force-version 6 ${HEADLESS+-s} -r "$world" &

# Launch the PX4 simulator.
env PX4_SIM_MODEL="$airframe" "$build_dir/bin/px4" -d -s etc/init.d-posix/rcS "$build_dir/etc"
