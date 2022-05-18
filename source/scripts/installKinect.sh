#!/bin/bash

#Installing rospackage openni

echo"Installing ROS Package OpenNI"
sudo apt-get install ros-hydro-openni-launch ros-hydro-openni-camera
echo "Complete..."

#Create tmp folder to install kinect for windows drivers
echo "Preparing to install \"Kinect for Windows\" drivers..."
echo "Creating tmp folder..."
mkdir libs/tmp
cd libs/

echo "Extracting libraries..."
tar -xvzf OpenNI-Bin-Dev-Linux-x86-v1.5.4.0.tar.gz -C tmp/
tar -xvzf Sensor-Bin-Linux-x86-v5.1.2.1.tar.gz -C tmp/

#Install unstable branch of OpenNI 1.5.4.0
echo "Unstable branch of OpenNI 1.5.4.0"
cd tmp/OpenNI-Bin-Dev-Linux-x86-v1.5.4.0/
sudo ./install.sh

#Install SensorKinecct drivers
echo "Unstable branch of SensorKinect 5.1.2.1"
cd ../Sensor-Bin-Linux-x86-v5.1.2.1/
sudo ./install.sh

echo "Installation complete!"
cd ../../..
rm -rf libs/tmp

exit 0

