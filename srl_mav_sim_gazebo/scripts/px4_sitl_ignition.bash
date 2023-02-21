#!/usr/bin/env bash
set -e

if [ "$#" -lt 3 ]
then
	printf 'usage: %s px4-repo-dir world mav\n' "${0##*/}"
	exit 2
fi

src_dir="$1"
world="$2"
mav="$3"

build_dir="$src_dir/build/px4_sitl_default/"
px4_sitl_bin="$build_dir/bin/px4"
workdir="$build_dir/tmp/rootfs"
mkdir -p "$workdir"

export PX4_SIM_MODEL="$mav"

# Kill processes that might stil be running from last time.
pkill -x px4 || true
pkill -x "px4_$mav" || true

source "$src_dir/Tools/setup_ignition.bash" "$src_dir" "$build_dir"
ign gazebo -v 4 --force-version 6 ${HEADLESS+-s} -r "${world}" &
# Kill Ignition Gazebo once the script exits.
# shellcheck disable=SC2064
trap "kill $! || true" EXIT

cd "$workdir" || exit 1
if [ "${INTERACTIVE+x}" ]
then
	INTERACTIVE=
fi
"$px4_sitl_bin" ${INTERACTIVE:--d} "$build_dir/etc" -s etc/init.d-posix/rcS -t "$src_dir/test_data"
