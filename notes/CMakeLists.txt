cmake_minimum_required(VERSION 2.8.3)
project(exceed_bot)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  #roscpp
  rospy
  sensor_msgs
  joy_teleop
  joy
)

#find_package(Eigen REQUIRED)
#find_package(PCL REQUIRED COMPONENTS common io)
#find_package(OpenCV REQUIRED)
#find_package(Boost REQUIRED)
#find_package(NUMPY REQUIRED)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES exceed_bot
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#)

#target_link_libraries(${PY_PROJECT_NAME}
#    ${catkin_LIBRARIES}
#    ${Boost_LIBRARIES}
#)
