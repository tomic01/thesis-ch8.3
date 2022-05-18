#!/bin/bash
#Author: Marco Barbosa (marco.barbosa@selftech.pt)
rm -rf build
mkdir build
cd build
cmake ../
make
FAKEROOT="`pwd`/fakeroot"
mkdir $FAKEROOT
make DESTDIR=$FAKEROOT install
cpack

