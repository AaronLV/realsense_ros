# realsense_ros
I rewrite the ROS node for realsense camera SR300 for solving the misalignment between RGB image and Depth image
* Used Realsense API: rs2::align
* Camera version: realsense SR300

When you use this ROS node, please tune two parameters according to your own realsense camera
You can locate the two parameters in realsense_ros_node.cpp line:45,46
```
#define  shift_x 2
#define  shift_y 8
```

To run this ros node
```
rosrun realsense_ros realsense_ros_node
```
