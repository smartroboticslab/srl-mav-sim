# Simulator for SRL MAVs

The simulators have been tested on Ubuntu 24.04 using ROS2 Jazzy.


## Setup

Install the `ros-jazzy-desktop` package by following the instructions
from [here](https://docs.ros.org/en/jazzy/Installation.html) and then install the
common dependencies.

``` sh
# MAVROS and catkin
sudo apt install ros-jazzy-depth-image-proc ros-jazzy-mavlink ros-jazzy-mavros ros-jazzy-mavros-msgs
# Other dependencies
sudo apt install libgflags-dev
```

Create a new ROS2 workspace.

``` sh
mkdir -p ~/srl_mav_sim_ws/src
cd ~/srl_mav_sim_ws
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

See [`srl_sim_gz`](srl_sim_gz/README.md) on how to
build and use the MAV simulator based on GZ. See
[`srl_mpc_examples`](srl_mpc_examples/README.md) for usage examples of the SRL
linear MPC.


## MAVROS

[MAVROS](https://docs.ros.org/en/jazzy/p/mavros/) allows communicating with the PX4 through
ROS. It is the interface used by SRL controllers.
