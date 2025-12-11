# The manuscript has been submitted to IEEE Transactions on Consumer Electronics

# Multi-Scale Registration of LiDAR Odometry Based on Vertical Constraints in Dynamic Environments

![](https://github.com/qfwang23/LO/blob/29e1d1d914cf2181cd0836f47ab8939cfbd14d3e/LO_png/main.png)
## Catalog
- [Introduction](#Introduction)
- [Demo](#Demo)
- [Dependence](#Dependence)
- [Install](#Install)
- [Rusult](#Rusult)
- [Acknowledgements ](#Acknowledgements)
 
## Introduction
 
As a key technology for autonomous navigation and positioning of mobile robots, the Light Detection and Ranging (LiDAR) odometry is widely adopted in autonomous driving. However, most existing methods are single-scale registration: when the initial pose is unreliable or there are a large number of dynamic targets and occlusions in the scene, optimization is prone to converge to a local optimum, resulting in a decrease in registration accuracy. Moreover, the rotational 3D LiDAR has a limited number of channels in the pitch direction and a large angular interval, which leads to a low vertical resolution for long-distance and insufficient height observability, also causing a decrease in registration accuracy. To address these issues, this paper proposes a multi-scale registration LiDAR odometry based on vertical constraints in dynamic environments: using motion prediction as the initial pose, first completing frame-to-frame registration, then using the result as the initial value to perform frame-to-local-map registration to obtain the final pose, thereby enhancing the reliability of the initial value and reducing the risk of local optimum. To suppress height drift, vertical constraints are introduced in the Iterative Closest Point (ICP) iteration interior: setting thresholds and limits for the height translation increment within the iteration, and updating preferentially based on the predicted benefit; in the iteration exterior, frame-level cumulative height displacement is again limited. Experiments on various scenarios from the public KITTI dataset show that the proposed method achieves better trajectory accuracy than mainstream methods, verifying the effectiveness and robustness of the method in dynamic environments.

## Demo
![示例图片](https://github.com/qfwang23/ALO/blob/6aa048cc49058d78d20e694bbfcd3c419e20cf9a/demo.gif)

## Dependence
```bash
Ubuntu 18.04 or 20.04
ROS Melodic（roscpp、std_msgs、sensor_msgs、geometry_msgs、pcl_ros）
C++ 14
CMake ≥ 3.16
PCL≥ 1.10.0
Eigen ≥ 3.3.7
```

## Install
 
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/qfwang23/LO.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslauch LO lo.launch
rosbag play [topic]
```

## Rusult

![示例图片](https://github.com/qfwang23/LO/blob/3eaeaeebb11cf3b1210e6e47297447f7a408c7fe/LO_png/tab1.png)

![示例图片](https://github.com/qfwang23/LO/blob/3eaeaeebb11cf3b1210e6e47297447f7a408c7fe/LO_png/traij.png)

## Acknowledgements
```bash
@article{vizzo2023ral,
  author    = {Vizzo, Ignacio and Guadagnino, Tiziano and Mersch, Benedikt and Wiesmann, Louis and Behley, Jens and Stachniss, Cyrill},
  title     = {{KISS-ICP: In Defense of Point-to-Point ICP -- Simple, Accurate, and Robust Registration If Done the Right Way}},
  journal   = {IEEE Robotics and Automation Letters (RA-L)},
  pages     = {1029--1036},
  doi       = {10.1109/LRA.2023.3236571},
  volume    = {8},
  number    = {2},
  year      = {2023}
}

@article{chen2022direct,
  author={Kenny Chen and Brett T. Lopez and Ali-akbar Agha-mohammadi and Ankur Mehta},
  journal={IEEE Robotics and Automation Letters}, 
  title={Direct LiDAR Odometry: Fast Localization With Dense Point Clouds}, 
  year={2022},
  volume={7},
  number={2},
  pages={2000-2007},
  doi={10.1109/LRA.2022.3142739}
}
```

