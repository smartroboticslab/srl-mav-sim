#!/bin/bash
set -e
source /opt/ros/humble/setup.bash

# Start MicroXRCEAgent
nohup MicroXRCEAgent udp4 -p 8888 > agent_log.txt 2>&1 &

# Start an interactive shell
/bin/bash
