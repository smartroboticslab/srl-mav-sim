#!/bin/bash
set -euo pipefail

WORLD_NAME=$1
GUI=$2

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

image="srl-mav-sim-image"

if docker image inspect "$image" >/dev/null 2>&1; then
    echo "Docker image '$image' already exists, skipping build."
else
    echo "Docker image '$image' not found. Locating build_docker.sh..."

    build_script=""

    if [[ "$script_dir" == */share/srl_sim_gz ]]; then
        candidate="$script_dir/../../../../src/srl-mav-sim/docker/build_docker.sh"
        if [ -f "$candidate" ]; then
            build_script="$(cd "$(dirname "$candidate")" && pwd)/$(basename "$candidate")"
        fi
    elif [[ "$script_dir" == */srl-mav-sim/docker ]]; then
        candidate="$script_dir/build_docker.sh"
        if [ -f "$candidate" ]; then
            build_script="$(cd "$(dirname "$candidate")" && pwd)/$(basename "$candidate")"
        fi
    fi

    if [ -z "$build_script" ] || [ ! -f "$build_script" ]; then
        echo "Error: build_docker.sh not found. Expected in one of the repository locations." >&2
        exit 1
    fi

    echo "Running build script: $build_script"
    (cd "$(dirname "$build_script")" && bash "$(basename "$build_script")")
fi

docker run --rm "$image" bash -c ". /ros2_ws/install/setup.bash && ros2 launch srl_sim_gz sim.launch.xml airframe:=rmf_owl world:=/ros2_ws/install/srl_sim_gz/share/srl_sim_gz/resources/worlds/${WORLD_NAME}.world gui:=${GUI}"