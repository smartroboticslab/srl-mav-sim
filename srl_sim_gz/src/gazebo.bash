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
world="$2"

export GZ_SIM_RESOURCE_PATH=$px4_dir/../../../srl_sim_gz/share/srl_sim_gz/resources/:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=$px4_dir/../../../srl_sim_gz/share/srl_sim_gz/:$GZ_SIM_SYSTEM_PLUGIN_PATH
gz sim ${HEADLESS+-s} -r $world
