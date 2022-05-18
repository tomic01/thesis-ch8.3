#!/bin/bash
ABSOLUTE_PATH=$(cd `dirname "${BASH_SOURCE[0]}"` && pwd)/`basename "${BASH_SOURCE[0]}"`
ABS_DIR=`dirname $ABSOLUTE_PATH`
ROOT_DIR=`echo $ABS_DIR | sed s%/launch%%`
CATKIN_WS=$ROOT_DIR/catkin_ws
ROSBUILD_WS=$ROOT_DIR/rosbuild_ws
echo "Root dir = $ROOT_DIR"
echo "Catkin workspace = $CATKIN_WS"
echo "Rosbuild workspace = $ROSBUILD_WS"

source $CATKIN_WS/devel/setup.bash
export ROS_PACKAGE_PATH=$ROSBUILD_WS:$ROS_PACKAGE_PATH

export MBOT_NAME=mcentral
roslaunch $ROOT_DIR/launch/mcentral.launch

