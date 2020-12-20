#!/bin/sh
#rosbag record /camera/color/image_raw/compressed /camera/aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info /camera/color/camera_info /static_tf
rosbag record /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info /camera/color/camera_info /static_tf /encoder /servo_pulse /motor_pulse /joy /left_switch /right_switch /cmd_vel
