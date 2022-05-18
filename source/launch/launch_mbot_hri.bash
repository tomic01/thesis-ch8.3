#!/bin/bash
ABSOLUTE_PATH=$(cd `dirname "${BASH_SOURCE[0]}"` && pwd)/`basename "${BASH_SOURCE[0]}"`
ABS_DIR=`dirname $ABSOLUTE_PATH`
ROOT_DIR=`echo $ABS_DIR | sed s%/launch%%`
CATKIN_WS=$ROOT_DIR/catkin_ws
ROSBUILD_WS=$ROOT_DIR/rosbuild_ws
echo "Root dir = $ROOT_DIR"
echo "Catkin workspace = $CATKIN_WS"
echo "Rosbuild workspace = $ROSBUILD_WS"

source $ROOT_DIR/catkin_ws/devel/setup.bash

export ROS_PACKAGE_PATH=$ROSBUILD_WS:$ROS_PACKAGE_PATH

export DISPLAY=:0
cd $ROOT_DIR/catkin_ws/src/rfid_node
sh hri_voices.sh &
roscd
cd ../../launch
#cd $ROOT_DIR/scripts
#./setupTouchScreen.sh
#cd $ROOT_DIR/launch
echo "Starting HRI Interfaces..."
roslaunch hri_interfaces_launcher hri_interfaces_tests.launch robot:=$MBOT_NAME &
sleep 10
echo "Starting Interaction Manager.."
roslaunch $ROOT_DIR/launch/mbot_hri.launch
