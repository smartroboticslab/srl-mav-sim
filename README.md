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

Alternatively, if you lazy like me you can run the entire simulator in docker
mode via three make targets:

```
make install_docker # Install docker
make build_docker   # Build docker container
make run_docker     # Run docker container
# Once you're running the container, both the username and password is `docker`
```

## Important Note

If the drone does not seem to be taking off in Gazebo, you will probably need
to set the following PX4 parameters
([source](https://github.com/PX4/PX4-Autopilot/issues/19919#issuecomment-1188864384)):

```
rosrun mavros mavparam set COM_RCL_EXCEPT 4  # RC LOSS EXCEPTION -> 4 (Not documented)
rosrun mavros mavparam set NAV_DLL_ACT 0     # GCS loss failsafe mode -> 0 (Disabled)
rosrun mavros mavparam set NAV_RCL_ACT 0     # RC loss failsafe modea -> 0 (Not documented)
```

Or simply type `make fix_px4` in the terminal. Then you can test out whether
the drone can take off by running:

```
python3 scripts/test_offboard_mode.py  # Sends offboard + arm + position setpoint command
```

## MAVROS

[MAVROS](http://wiki.ros.org/mavros) allows communicating with the PX4 through
ROS. It is the interface used by SRL controllers.
