#!/bin/sh

# Raspberry pi's master is a230-pc41, which controls cameras, video and logs recording
export ROS_IP=172.22.62.209
export ROS_MASTER_URI=http://a230-pc89:11311
export ROSLAUNCH_SSH_UNKNOWN=1
. ~/roscvimage_catkinws/devel/setup.sh
exec "$@"
