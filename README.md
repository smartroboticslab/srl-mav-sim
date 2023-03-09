# Simulators for SRL MAVs

The simulators have been tested on Ubuntu 20.04 using ROS Noetic.


## Setup

Install the `ros-noetic-desktop-full` package by following the instructions
from [here](http://wiki.ros.org/noetic/Installation) and then install the
common dependencies.

``` sh
# MAVROS and catkin
sudo apt install ros-noetic-mavlink ros-noetic-mavros ros-noetic-mavros-msgs python3-catkin-tools
```

Create a new ROS workspace.

``` sh
mkdir -p ~/srl_mav_sim_ws/src
cd ~/srl_mav_sim_ws
catkin init
```

Clone this repository and all submodules.

``` sh
cd src
git clone --recurse-submodules git@bitbucket.org:smartroboticslab/srl-mav-sim.git
cd srl-mav-sim
# Or if you didn't add --recurse-submodules when cloning run the following:
git submodule update --init --recursive
```

Install the PX4 dependencies.

``` sh
# PX4-Autopilot dependencies
./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
```

### MAVROS

MAVROS allows communicating with the PX4 through ROS. It is the interface used
by SRL controllers.


## Usage

See the individual package READMEs for details on usage:

* [`srl_sim_gazebo_classic`](srl_sim_gazebo_classic/README.md): MAV simulator
  based on Gazebo Classic.
* [`srl_sim_gazebo_ignition`](srl_sim_gazebo_ignition/README.md): MAV simulator
  based on Ignition Gazebo.
* [`srl_mpc_examples`](srl_mpc_examples/README.md): usage examples for the SRL
  linear MPC.

**IMPORTANT** when running the simulator, you must have QGroundControl in the background to disable some Failsafe modes. It will be a WIP to deactivate this failsafe mode. To download QGroundControl please follow the instructions from [here](http://qgroundcontrol.com/). To set up the virtual joysticks you have to go to: QGroundControl symbol (top left corner of QGroundControl) >> ApplicationSettings >> General >> tick the virtual joystick condiguration. 


## TODO

* Create a SDF files for our MAVs?
