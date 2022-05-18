#!/bin/bash
#Install required libraries

echo "Installing required libraries..."

sudo apt-get install chrony ssh subversion python-pip python-numpy python-scipy libmxml-dev 

sudo apt-get install htop screen emacs ipython python-matplotlib python-matplotlib python-wxgtk2.8

sudo apt-get install libasound2-dev tap-plugins

sudo apt-get install libopenni-dev

sudo easy_install -U distribute
sudo pip install -U scikit-image
sudo pip install -U colorama

sudo apt-get install sox curl mplayer libsox-fmt-mp3

sudo apt-get remove modem-manager

sudo apt-get install libgstreamer0.10-0 libgstreamer0.10-dev libgstreamer-plugins-base0.10-0 libgstreamer-plugins-base0.10-dev libphonon4 libphonon-dev libphononexperimental-dev libphononexperimental4 automoc gstreamer0.10-plugins-bad

echo "Installation complete!"

echo "Configuring timezone..."

sudo dpkg-reconfigure tzdata
sudo locale-gen pt_PT pt_PT.UTF-8
export LC_ALL="en_US.UTF-8"
sudo locale-gen en_US en_US.UTF-8
sudo dpkg-reconfigure locales

echo "Configuration complete!"

echo "Installing other software..."

sudo pip install scikit-fmm
sudo apt-get install sox
sudo usermod -a -G audio monarch

echo "Instalation complete!"

echo "Installing other dependencies..."
cd ../catkin_ws/src
git clone https://github.com/UC3MSocialRobots/rospy_utils

echo "Finish..."

exit 0

