#!/bin/bash
# External, ROS and system package dependencies
sudo apt-get install git cmake build-essential
sudo apt-get install freeglut3-dev libxmu-dev libxi-dev
sudo apt-get install libudev-dev

cd ../catkin_ws/src/monarch_perception/tools/openni2/libusb-1.0.18/

./configure --prefix=/usr --disable-static
make
sudo make install

sudo updatedb 

cd ../

sudo apt-get install git-core cmake pkg-config build-essential libxmu-dev libxi-dev
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect/
mkdir build
cd build/
cmake ..

sudo apt-get install cmake-curses-gui

echo -e "\n\n******************************************\nLIBUSB_1_LIBRARY path:\n"
locate libusb-1.0.so  #(to find the LIBUSB_1_LIBRARY path)

echo -e "\n******************************************\nWithin the ccmake gui:\nset BUILD_OPENNI2_DRIVER = ON\nset LIBUSB_1_LIBRARY = path above (ex: ../catkin_ws/src/monarch_perception/tools/openni2/libusb-1.0.18/libusb/.libs/libusb-1.0.so)\n******************************************\n"
read -p "Copy the LIBUSB_1_LIBRATY path and press any key to start ccmake gui..."

ccmake ..
#Within the ccmake gui set:
#BUILD_OPENNI2_DRIVER --> should be ON
#SET -> LIBUSB_1_LIBRARY ~/sandbox/monarch/code/trunk/catkin_ws/src/monarch_perception/tools/openni2/libusb-1.0.18/libusb/.libs/libusb-1.0.so (to the found library path using locate)

make
sudo make install
sudo ldconfig /usr/local/lib/

#---edit your bashrc to add LD_LIB
echo -e "\n\n******************************************\nCopy the following line in the .bashrc file:\nexport LD_LIBRARY_PATH=YOUR_DIR/catkin_ws/src/monarch_perception/tools/openni2/libfreenect/build/lib/:$LD_LIBRARY_PATH\n"
read -p "Press any key to continue..."

source ~/.bashrc

cd ../../../openni2/

echo -e "\nExtracting libraries..."

tar -xjf OpenNI-Linux-x86-2.2.tar.bz2
tar -xjf NiTE-Linux-x86-2.2.tar.bz2

cd OpenNI-Linux-x86-2.2
sudo ./install.sh
cd ..

cd NiTE-Linux-x86-2.2
sudo ./install.sh
cd ..

cp -L libfreenect/build/lib/OpenNI2-FreenectDriver/libFreenectDriver.so OpenNI-Linux-x86-2.2/Samples/Bin/OpenNI2/Drivers/

cp -L libfreenect/build/lib/OpenNI2-FreenectDriver/libFreenectDriver.so OpenNI-Linux-x86-2.2/Redist/OpenNI2/Drivers/

cp -L libfreenect/build/lib/OpenNI2-FreenectDriver/libFreenectDriver.so NiTE-Linux-x86-2.2/Samples/Bin/OpenNI2/Drivers/

echo -e "\nInstalation complete."