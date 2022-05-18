#!/bin/bash
ABSOLUTE_PATH=$(cd `dirname "${BASH_SOURCE[0]}"` && pwd)/`basename "${BASH_SOURCE[0]}"`
ABS_DIR=`dirname $ABSOLUTE_PATH`
ROOT_DIR=`echo $ABS_DIR | sed s%/launch%%`
CATKIN_WS=$ROOT_DIR/catkin_ws
ROSBUILD_WS=$ROOT_DIR/rosbuild_ws
echo "Root dir = $ROOT_DIR"
echo "Catkin workspace = $CATKIN_WS"
echo "Rosbuild workspace = $ROSBUILD_WS"
echo "MBOT_NAME workspace = $MBOT_NAME"

source $ROOT_DIR/catkin_ws/devel/setup.bash

cd $ROOT_DIR/launch

export ROS_PACKAGE_PATH=$ROSBUILD_WS:$ROS_PACKAGE_PATH

roslaunch $ROOT_DIR/launch/mbot_nav.launch

