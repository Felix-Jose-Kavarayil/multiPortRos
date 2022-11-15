#!/bin/sh

# Raspberry pi's master is a230-pc41, which controls cameras, video and logs recording
export ROS_IP=172.22.24.91
export ROS_MASTER_URI=http://a230-pc005:11311
export ROSLAUNCH_SSH_UNKNOWN=1
. ~/catkin_ws/devel/setup.sh
exec "$@"
