# `srl_sim_gazebo_classic`

A simulator for SRL MAVs using PX4 and Gazebo Classic. It has been tested on
Ubuntu 20.04 and ROS Noetic.


## Building

Install dependencies, setup the ROS workspace and clone all submodules as
described in the [main README](../README.md). Then install the dependencies for
the `srl_sim_gazebo_classic` ROS package.

``` sh
# PX4-SITL_gazebo-classic dependencies
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

Build the package.

``` sh
catkin build -DCMAKE_BUILD_TYPE=Release srl_sim_gazebo_classic
```


## Usage

Source devel/setup.bash from the ROS workspace root and run:

``` sh
roslaunch srl_sim_gazebo_classic sim.launch
```

This will launch Gazebo Classic with the PX4 simulator and
[MAVROS](http://wiki.ros.org/mavros). See
[`launch/sim.launch`](launch/sim.launch) for the launch file arguments.

MAVROS can be used to send commands to the MAV and is the interface the SRL
controllers use.

Depending on the MAV model loaded sensor messages might be available over ROS,
e.g. for the default Iris MAV there should be a topic named
`/iris/camera/depth/image_raw` with the depth camera images.
