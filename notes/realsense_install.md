2020-08-02

- on 18.04, desktop.
- using catkin_make_isolated. (catkin_tools from git master also seems to work). python stuff is put into "2.7" directory but with conda still seems fine
- install realsense2 dev from intel realsense .deb, not main debian
- clone the ros realsense2 package into src/
- if cv+bridge complains about /usr/share/include/opencv, simply symlinking /usr/share/include/opencv4 to /usr/share/include/opencv works?
- catkin_make_isolated (--install)
- then source devel also at least runs node
