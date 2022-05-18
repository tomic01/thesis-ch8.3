cmake_minimum_required(VERSION 2.8.3)
project(monarch_msgs)

# set install prefix before anything else
set(CMAKE_INSTALL_PREFIX "/opt/monarch_msgs")


# Find catkin macros and libraries
# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS genmsg message_generation geometry_msgs sensor_msgs std_msgs actionlib_msgs move_base_msgs)

# System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


# Uncomment this if the package has a setup.py. This macro ensures
# modules and global scripts declared therein get installed
# See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# To declare and build messages, services or actions from within this
# package, follow these steps:
# * Let MSG_DEP_SET be the set of packages whose message types you use in
# your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# * In the file package.xml:
# * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
# * If MSG_DEP_SET isn't empty the following dependencies might have been
# pulled in transitively but can be declared for certainty nonetheless:
# * add a build_depend tag for "message_generation"
# * add a run_depend tag for "message_runtime"
# * In this file (CMakeLists.txt):
# * add "message_generation" and every package in MSG_DEP_SET to
# find_package(catkin REQUIRED COMPONENTS ...)
# * add "message_runtime" and every package in MSG_DEP_SET to
# catkin_package(CATKIN_DEPENDS ...)
# * uncomment the add_*_files sections below as needed
# and list every .msg/.srv/.action file to be processed
# * uncomment the generate_messages entry below
# * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder


add_message_files(
    FILES
    ArmsControl.msg
    AuxiliaryBatteriesVoltage.msg
    BatteriesVoltage.msg
    BehaviorRequestLog.msg
    BehaviorSelection.msg
    BehaviorSelectionArray.msg
    BumpersReadings.msg
    CapacitiveSensorsReadings.msg
    ChargerStatus.msg
    EyesControl.msg
    FourWheelOmniDelta.msg
    GameGoal.msg
    GameHriGoal.msg
    GamePlayerStatus.msg
    GameTurnSummary.msg
    GestureExpression.msg
    GroundSensorsReadings.msg
    HeadControl.msg
    HardstopStatus.msg
    InteractionExecutorStatus.msg
    KeyValuePair.msg
    KeyValuePairArray.msg
    LedControl.msg
    MotorBoardCommunicationStatusReadings.msg
    MotorBoardTemperatures.msg
    MotorBoardVoltages.msg
    MotorsCoolingFans.msg
    MouthLedControl.msg
    MouthPictureLedControl.msg
    MouthStringLedControl.msg
    MouthVumeterControl.msg
    NonVerbalSound.msg
    PersonFrontofRobot.msg
    PersonFrontRobotArray.msg
    PersonLocalizationTrackingData.msg
    PersonLocalizationTrackingDataArray.msg
    mcmc.msg
    mcmcArray.msg
    loc.msg
    locs.msg
    PlayAudioControl.msg
    PlayVideoControl.msg
    RecognizedSpeech.msg
    RfidReading.msg
    ScreenButton.msg
    ScreenInterfaceInputConfig.msg
    SensorBoardCommunicationStatusReadings.msg
    SetMotorBoardCoolingFans.msg
    SetStateAuxiliaryPowerBattery.msg
    SetStateElectronicPower.msg
    SetStateImu.msg
    SetStateMotorsPower.msg
    SetStateSonars.msg
    Utterance.msg
    VideoProjectorControl.msg
    PersonLocalizationTrackingParticle.msg
    PersonLocalizationTrackingParticleArray.msg
    GaussianCostmap.msg    
    GaussianCostmapArray.msg
)

# Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
    actionlib_msgs
    move_base_msgs
)

###################################
## catkin specific configuration ##
###################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if you package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects
# also need
catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES monarch_msgs
    CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs std_msgs move_base_msgs
    #  DEPENDS
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
# include_directories(include)

# Declare a cpp library
# add_library(monarch_msgs
#   src/${PROJECT_NAME}/monarch_msgs.cpp
# )

# Declare a cpp executable
# add_executable(monarch_msgs_node src/monarch_msgs_node.cpp)

# Add cmake target dependencies of the executable/library
# as an example, message headers may need to be generated before nodes
# add_dependencies(monarch_msgs_node monarch_msgs_generate_messages_cpp)

# Specify libraries to link a library or executable target against
# target_link_libraries(monarch_msgs_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark executables and/or libraries for installation
# install(TARGETS monarch_msgs monarch_msgs_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
# myfile1
# myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_monarch_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# Add folders to be run by python nosetests
catkin_add_nosetests(src/test/test_kvpa_utils.py)

###########
## CPack ##
###########

SET(CPACK_PACKAGING_INSTALL_PREFIX "/")
SET(CPACK_PACKAGE_DEFAULT_LOCATION "/opt/${CPACK_PACKAGE_NAME}")
SET(CPACK_SET_DESTDIR true)
SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Marco Barbosa")  # required
SET(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "i386")
SET(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Monarch messages.")
SET(CPACK_DEBIAN_PACKAGE_HOMEPAGE "http://monarch-fp7.eu/")
SET(CPACK_PACKAGE_VERSION_MAJOR "0")
SET(CPACK_PACKAGE_VERSION_MINOR "1")
SET(CPACK_PACKAGE_VERSION_PATCH "4")

INCLUDE(CPack)
