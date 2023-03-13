# `srl_sim_gazebo_ignition`

A simulator for SRL MAVs using PX4 and Ignition Gazebo. It has been tested on
Ubuntu 20.04, ROS Noetic and Ignition Gazebo Fortress.


## Building

Install dependencies, setup the ROS workspace and clone all submodules as
described in the [main README](../README.md). Install Ignition Gazebo Fortress
using [these](https://gazebosim.org/docs/fortress/install) instructions. Then
build the `srl_sim_gazebo_ignition` ROS package.

``` sh
env IGNITION_VERSION=fortress catkin build -DCMAKE_BUILD_TYPE=Release srl_sim_gazebo_ignition ros_ign digiforest_simulation
```


## Usage

Source devel/setup.bash from the ROS workspace root and run:

``` sh
roslaunch srl_sim_gazebo_ignition sim.launch
```

# SRL MAV simulators using PX4 and Gazebo

In order to launch your simulation with ignition gazebo, there are some launch files which have been added to this repo and which demonstrate the usage of the simulation environment. 

The most basic functionality is sim.launch which contains all the commands to be able to run the simulator and to have the necessary topics being published for interaction with ROS. Then, there is also digiforest_sim.launch which is a wrapper of sim.launch which enhances the functionality by defining which robot and which environment to load to the simulator and it also launches MAVROS.

## Build you own MAVs and world files

In order to be able to have a more modular simulator we will describe here how to build your own environment with the Ignition Gazebo stack. **PLEASE READ THIS CAREFULLY** the amount of documnetation provided vy PX4 is scarce and outdated, it has been a lot of reverse engineering and trial and error to have the simulator working with our own setup and this might help you save several hours until you have all working as you wish.

As for the base of this information, we will use the [rmf_owl robot](../PX4-Autopilot/Tools/simulation-ignition/models/rmf_owl/model.sdf) and the [depot.world](../PX4-Autopilot/Tools/simulation-ignition/worlds/depot.world), which are the setups we created for our experiments. The rmf_owl just contains all the required sensors and plugins which are needed to have it working in a normal Ignition Gazebo simulator environment. All the requirements needed from PX4's side are added in the depot.world file, when we include the rmf_owl drone into the world. 

The main things which are needed are: 

* Adding the libmavlink_sitl_ign_gazebo.so plugin. This plugin enables communication between Ignition Gazebo and Mavlink. As the parameters of the plugin, they are specifying how to connect via udp protocol Mavlink and Ignition Gazebo. Also, it includes the information of the topics where the magnetometer, the barometer and the IMU are publishing the messages into. **Very important**: the imu included in YOUR ROBOT must publish to the same topic as specified in the plugin in the tag ```xml </imuSubTopic>```. Else, failsafe modes will be activated and you will not be able to launch the drone. 

* Adding the libgazebo_barometer_plugin, libgazebo_magnetometer_plugin and the libgazebo_gps_plugin plugins. These are needed to specify QGroundControl the simulated position of the drone. If this is not done, the failsafe will be activated and the drone will not fly. They are attached to the body of the drone. 

* Be careful, you need to be able to produce a realtime simulator so PX4 does not considers that there is lost of connectivity with the drone. For example, if it takes a long time for PX4 to receive a ping from the simulator's side, it will consider that the drone has disconnected and it will eventually lead to not having a working simulation. For us, the main issues were the collisions being too complex, as it was following the shape defined by a .dae file. When changing the collisions of our drone to a box, it worked directly out of the box in real time. 
