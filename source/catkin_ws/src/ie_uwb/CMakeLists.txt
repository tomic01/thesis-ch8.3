cmake_minimum_required(VERSION 2.8.3)
project(ie_uwb)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

#catkin_python_setup()

add_message_files(FILES PointWithCovariance.msg uwb.msg uwb_array.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
