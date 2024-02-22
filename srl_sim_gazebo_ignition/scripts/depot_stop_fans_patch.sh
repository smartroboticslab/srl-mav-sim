#!/bin/sh
# SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0
set -eu

sdf="$HOME/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/depot/5/model.sdf"
sdf_original_sha256='11ad26026c49db0b73be882d3f61c3ab5fc637ca736a657d20b516b14b951afe'

usage() {
	printf 'Usage: %s [-R|-h]\n' "${0##*/}"
	printf 'Modify the depot environment SDF file stored in ~/.ignition\n'
	printf 'to stop the fans from rotating.\n'
	printf -- '  -R  Revert the patch to re-enable the fans.\n'
	printf -- '  -h  Show this help message.\n'
}

depot_patch() {
	cat <<- 'EOF'
	--- model.sdf.bak	2024-02-22 13:19:49.086417709 +0100
	+++ model.sdf	2024-02-22 13:20:20.334075587 +0100
	@@ -514,27 +514,19 @@
	             <parent>world</parent>
	             <child>main</child>
	         </joint>
	-        <joint name="fan1_joint" type="revolute">
	+        <joint name="fan1_joint" type="fixed">
	             <parent>main</parent>
	             <child>Fan1</child>
	             <axis>
	                 <xyz>0 0 1</xyz>
	             </axis>
	         </joint>
	-        <joint name="fan2_joint" type="revolute">
	+        <joint name="fan2_joint" type="fixed">
	             <parent>main</parent>
	             <child>Fan2</child>
	             <axis>
	                 <xyz>0 0 1</xyz>
	             </axis>
	         </joint>
	-        <plugin filename="ignition-gazebo-joint-controller-system" name="ignition::gazebo::systems::JointController">
	-            <joint_name>fan1_joint</joint_name>
	-            <initial_velocity>1.0</initial_velocity>
	-        </plugin>
	-        <plugin filename="ignition-gazebo-joint-controller-system" name="ignition::gazebo::systems::JointController">
	-            <joint_name>fan2_joint</joint_name>
	-            <initial_velocity>2.0</initial_velocity>
	-        </plugin>
	     </model>
	 </sdf>
	EOF
}

revert=0
while getopts 'Rh' option
do
	case "$option" in
		R)
			revert=1
			;;
		h)
			usage
			exit 0
			;;
		*)
			usage
			exit 2
			;;
	esac
done
shift "$((OPTIND - 1))"

sdf_sha256=$(sha256sum "$sdf" | awk '{ print $1 }')
if [ -z "$sdf_sha256" ]
then
	exit 1
fi

if [ "$revert" -eq 0 ]
then
	if [ "$sdf_sha256" = "$sdf_original_sha256" ]
	then
		cp "$sdf" "$sdf.bak"
		depot_patch | patch "$sdf"
	else
		printf 'Patch already applied\n'
	fi
elif [ "$revert" -eq 1 ]
then
	if [ "$sdf_sha256" = "$sdf_original_sha256" ] 
	then
		rm -f "$sdf.bak"
		printf 'Patch already reverted\n'
	else
		depot_patch | patch -R "$sdf"
	fi
fi
