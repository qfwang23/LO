## Dependencies
-ROS 1 (Noetic): roscpp, tf2_ros, geometry_msgs, sensor_msgs, nav_msgs
-Eigen3 (libeigen3-dev)
TBB (libtbb-dev)
PCL (libpcl-dev)
Sophus (libsophus-dev)
tsl::robin_map
C++17 toolchain

## build and run
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/qfwang23/LO.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslauch LO lo.launch
