#!/bin/sh
export ROS_IP=172.22.50.23
export ROS_MASTER_URI=http://localhost:11311
export ROSLAUNCH_SSH_UNKNOWN=1
. ~/catkin_ws/devel/setup.sh
exec "$@"
