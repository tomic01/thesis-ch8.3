  _____           _        _ _    _____           _       _       
 |_   _|         | |      | | |  / ____|         (_)     | |      
   | |  _ __  ___| |_ __ _| | | | (___   ___ _ __ _ _ __ | |_ ___ 
   | | | '_ \/ __| __/ _` | | |  \___ \ / __| '__| | '_ \| __/ __|
  _| |_| | | \__ \ || (_| | | |  ____) | (__| |  | | |_) | |_\__ \
 |_____|_| |_|___/\__\__,_|_|_| |_____/ \___|_|  |_| .__/ \__|___/
                                                   | |            
                                                   |_|            


In order to properly install the required libraries and dependencies please follow this readme:

First install the dlib library:
   - sudo apt-get install libopenblas-dev liblapack-dev
   - download dlib from "http://dlib.net"
   - tar xf dlib*
   - cd dlib*
   - sudo cp −R dlib /usr/local/include

Next run the following scripts:
   - sudo ./installROSHydro.sh
   - sudo ./install_re2
   - sudo ./installLibraries.sh
   - sudo ./install_hri_interfaces_deps.sh

Additional dependencies:
   - cd ../catkin_ws/src
   - git clone https://github.com/UC3MSocialRobots/rospy_utils

Script to install "Kinect for Windows" Ubuntu 12.04 LTS:
   - sudo ./installKinect.sh

Script to install Openni2:
   - sudo ./install_non-psi-openni2.sh

Script to configure mbot Touch Screen Ubuntu 12.04 LTS:
   - run the script automatically when the system starts

