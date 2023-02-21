#!/usr/bin/env bash
px4_dir="$1"
world="$2"
airframe="$3"
build_dir="$px4_dir/build/px4_sitl_default"

source "$px4_dir/Tools/setup_ignition.bash" "$px4_dir" "$build_dir"
ign gazebo --force-version 6 ${HEADLESS+-s} -r "$world" &
# Kill Ignition Gazebo once the script exits.
# shellcheck disable=SC2064
trap "kill $! || true" EXIT

env PX4_SIM_MODEL="$airframe" "$build_dir/bin/px4" -d -s etc/init.d-posix/rcS "$build_dir/etc"
