#!/bin/bash
ROS_VERSION=humble
COLCON_WS=$HOME/px4_ws
COLCON_SRC=$COLCON_WS/src

# SETUP
mkdir -p $COLCON_SRC

# BUILD PX4
cd $COLCON_WS
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot \
  && bash ./Tools/setup/ubuntu.sh \
  && make px4_sitl

# BUILD MICRO-XRCE-DDS-AGENT
cd $COLCON_WS
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
  && cd Micro-XRCE-DDS-Agent \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make \
  && sudo make install \
  && sudo ldconfig /usr/local/lib/

# BUILD COLCON WORKSPACE
cd $COLCON_WS
source /opt/ros/$ROS_VERSION/setup.bash && colcon build
