#!/bin/sh

HRI_DIR="../catkin_ws/src/hri_interfaces"

sh $HRI_DIR/ad_core/scripts/rosdep.sh
sh $HRI_DIR/asr/scripts/rosdep.sh
sh $HRI_DIR/etts/scripts/rosdep.sh
sh $HRI_DIR/mouth/scripts/rosdep.sh
sh $HRI_DIR/utils/scripts/rosdep.sh
