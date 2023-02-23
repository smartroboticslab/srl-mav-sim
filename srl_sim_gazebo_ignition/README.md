# `srl_sim_gazebo_ignition`

A simulator for SRL MAVs using PX4 and Ignition Gazebo. It has been tested on
Ubuntu 20.04, ROS Noetic and Ignition Gazebo Fortress.


## Building

Install dependencies, setup the ROS workspace and clone all submodules as
described in the [main README](../README.md). Install Ignition Gazebo Fortress
using [these](https://gazebosim.org/docs/fortress/install) instructions. Then
build the `srl_sim_gazebo_ignition` ROS package.

``` sh
catkin build -DCMAKE_BUILD_TYPE=Release srl_sim_gazebo_ignition
```


## Usage

Source devel/setup.bash from the ROS workspace root and run:

``` sh
roslaunch srl_sim_gazebo_ignition sim.launch
```

TODO
