# 1 = default, 0 = custom
rosrun dynamic_reconfigure dynparam set /camera/stereo_module visual_preset 1
## depth auto exposure off
# rosrun dynamic_reconfigure dynparam set /camera/stereo_module enable_auto_exposure 0
## 0 = off, 1 = on, 2 = auto
rosrun dynamic_reconfigure dynparam set /camera/stereo_module emitter_enabled 1
## exposure in microsecond units
# rosrun dynamic_reconfigure dynparam set /camera/stereo_module exposure 8500
## rgb auto exposure off
# rosrun dynamic_reconfigure dynparam set /camera/rgb_camera enable_auto_exposure 0
## rgb exposure in units of 0.1 ms (?) so 220 -> 22.0 ms. note: 1/150 is 66 ms, so 660; 1/200 is 50 ms, so 500
# rosrun dynamic_reconfigure dynparam set /camera/rgb_camera exposure 220
