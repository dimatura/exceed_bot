cmake_minimum_required(VERSION 2.8.3)
project(exceed_bot)

find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    std_msgs
    # cv_bridge
    image_transport
    tf
    )

catkin_python_setup()

catkin_package()
