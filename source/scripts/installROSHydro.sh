#!/bin/bash
#Install ROS hydro

echo "Installing ROS-Hydro..."

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list' 
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-hydro-desktop-full
sudo apt-get install ros-hydro-ros-base

echo "Installing ROS Packages..."

sudo apt-get install ros-hydro-dynamic-reconfigure ros-hydro-hokuyo-node ros-hydro-navigation ros-hydro-pocketsphinx ros-hydro-joystick-drivers ros-hydro-rosbridge-suite ros-hydro-mjpeg-server ros-hydro-usb-cam ros-hydro-rosjava ros-hydro-rosjava-build-tools ros-hydro-rosjava-bootstrap ros-hydro-gmapping ros-hydro-smach ros-hydro-smach-ros ros-hydro-audio-common-msgs ros-hydro-openni-launch ros-hydro-openni-camera ros-hydro-openni2-launch ros-hydro-openni2-camera ros-hydro-orocos-kdl ros-hydro-python-orocos-kdl ros-hydro-qt-build

sudo rosdep init
rosdep update
sudo apt-get install python-rosinstall

echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Finish..."


exit 0

