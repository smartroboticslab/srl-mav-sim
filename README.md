# Simulators for SRL MAVs

Make sure to clone all submodules either by passing the `--recurse-submodules`
flag to `git clone` or by running `git submodule update --init --recursive`
after cloning.

## Dependencies

# Ignition Gazebo

The simulator stack is based on Ignition Gazebo. The version used which supports ROS and the sensors which have been added to the simulator is Ignition Gazebo Fortress. In order to download Ignition Gazebo Fortress please do so [here](https://gazebosim.org/docs/fortress/install).

Once this is done, the next step consists in downloading the Ignition Gazebo to ROS bridges to be able to communicate with our stack. This can be done [here](https://github.com/gazebosim/ros_gz/tree/noetic). The correct version is the noetic branch as we are currently developing in the lab with this ROS distro and it allows to bridge against the Ignition Fortress simulation environment. 

# MAVROS

Another important package to install is MAVROS as this is the ROS package we are using to enable communication between ROS and Mavlink/PX4. The Mavros package source files are found [here](https://github.com/mavlink/mavros/tree/1.15.0). Please ensure to checkout to version 1.15.0 as this is the last release of the repository before switching to ROS2 releases. 

Mavros also requires to collect Mavlink for the communication with the PX4, this can be found here

TODO

``` sh
pip3 install --user kconfiglib
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libignition-gazebo3-dev
# + ROS stuff
# TODO
```

## TODO

* Allow switching to offboard mode without a joystick (e.g. through
  QGroundControl).
* Create a geometry file for the RMF_OWL for PX4 
* Add test scripts for the linear and non-linear MPCs.
* Modify the airframes according to our MAVs.
* Create an SDF file for our MAV?
