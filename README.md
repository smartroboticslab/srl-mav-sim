# Simulator for SRL MAVs

The simulators have been tested on Ubuntu 20.04 using ROS Noetic.


## Setup

Install the `ros-noetic-desktop-full` package by following the instructions
from [here](http://wiki.ros.org/noetic/Installation) and then install the
common dependencies.

``` sh
# MAVROS and catkin
sudo apt install ros-noetic-depth-image-proc ros-noetic-mavlink ros-noetic-mavros ros-noetic-mavros-msgs python3-catkin-tools
# Other dependencies
sudo apt install libgflags-dev
```

Create a new ROS workspace.

``` sh
mkdir -p ~/srl_mav_sim_ws/src
cd ~/srl_mav_sim_ws
catkin init
```

Clone this repository and all submodules.

``` sh
cd ~/srl_mav_sim_ws/src
git clone --recurse-submodules git@bitbucket.org:smartroboticslab/srl-mav-sim.git
cd srl-mav-sim
# Or if you didn't add --recurse-submodules when cloning run the following:
git submodule update --init --recursive
```

Install the PX4 dependencies.

``` sh
# PX4-Autopilot dependencies
./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
```

See [`srl_sim_gazebo_ignition`](srl_sim_gazebo_ignition/README.md) on how to
build and use the MAV simulator based on Ignition Gazebo. See
[`srl_mpc_examples`](srl_mpc_examples/README.md) for usage examples of the SRL
linear MPC.


## MAVROS

[MAVROS](http://wiki.ros.org/mavros) allows communicating with the PX4 through
ROS. It is the interface used by SRL controllers.
