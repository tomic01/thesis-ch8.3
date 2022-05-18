#!/bin/bash

# The version of Stage included in ROS has a bug on the odometry
# estimation published on /tf concerning lateral movement. This patch
# solves this but requires patching and re-building Stage.

# Instructions:
# 1. clone the Stage git repository, e.g.,
#       $ git clone https://github.com/ros-gbp/stage-release.git
# 2. switch to the appropriate ROS release branch, e.g.,
#       $ cd stage-release
#       $ git checkout remotes/origin/release/hydro/stage
# 3. apply the patch using, e.g.,
#       $ patch -p1 < ~/monarch/trunk/scripts/stage-release-hydro.patch 
# 3. compile and install, e.g.,
#       $ mkdir build
#       $ cd build
#       $ cmake -DCMAKE_INSTALL_PREFIX=~/stage ..
#       $ make
#       $ make install
# 4. set LD_LIBRARY_PATH to the lib directory at the installation directory, e.g.,
#    add the following line to your .bashrc:
#       export LD_LIBRARY_PATH=~/stage/lib:$LD_LIBRARY_PATH


# Date: 1-Oct-2014

# Rodrigo Ventura <rodrigo.ventura@isr.ist.utl.pt>
# Institute for Systems and Robotics
# Instituto Superior Técnico
# Lisbon, Portugal
# http://users.isr.ist.utl.pt/~yoda


if [ -z "$1" ]
  then
    echo "Please, enter the path to the patch file"
    exit 0
fi

echo "cd /tmp/"
cd /tmp/
git clone https://github.com/ros-gbp/stage-release.git

cd stage-release
git checkout remotes/origin/release/hydro/stage

patch -p1 < $1 

mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=~/stage ..

make
make install

echo "export LD_LIBRARY_PATH=~/stage/lib:$LD_LIBRARY_PATH" >> ~/.bashrc