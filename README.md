# bebop_control

This code represents ROS package used for calculation of mutual postions of ArUco markers and navigation of Parrot Bebop drone along a grid of such markers. You can start using the package by adding it to your conventional ROS catking workspace:
```
cd ~/catkin_ws/src
git clone https://github.com/vasilya93/bebop_control.git
cd ..
catkin_make
catkin_make install
```
Please note that this package makes use of other ROS packages [aruco_ros](http://wiki.ros.org/aruco) and [bebop_autonomy](http://wiki.ros.org/bebop_autonomy), so please install those packages in advance and make sure that example launch files from those packages can be run correctly on your computer. Once that is done, you can try using `bebop_control` by running
```
roslaunch bebop_control localization.launch
```
From ROS topic `bebop_pose` you must be able to read pose of the copter with respect to your ArUco markers.
