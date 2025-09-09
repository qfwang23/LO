#pragma once

#include "LO/pipeline/LO.hpp"

// ROS
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace lo_ros {

class OdometryServer {
public:
    OdometryServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

    void SaveGlobalMapAsPCD(const std::string& filename) const;

    void PrintStatistics() const;


private:

    void RegisterFrame(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void SavePoseToTUMFormat(const Sophus::SE3d &pose,
                             const ros::Time &timestamp,
                             const std::string &filename) const;

    void PublishOdometry(const Sophus::SE3d &pose,
                         const ros::Time &stamp,
                         const std::string &cloud_frame_id);

    void PublishClouds(const std::vector<Eigen::Vector3d> frame,
                       const std::vector<Eigen::Vector3d> keypoints,
                       const ros::Time &stamp,
                       const std::string &cloud_frame_id);

    Sophus::SE3d LookupTransform(const std::string &target_frame,
                                 const std::string &source_frame) const;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    int queue_size_{10};

    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    bool publish_odom_tf_;
    bool publish_debug_clouds_;

    ros::Subscriber pointcloud_sub_;

    ros::Publisher odom_publisher_;
    ros::Publisher frame_publisher_;
    ros::Publisher kpoints_publisher_;
    ros::Publisher map_publisher_;
    ros::Publisher traj_publisher_;
    nav_msgs::Path path_msg_;

    ros::Publisher global_map_publisher_;

    lo::pipeline::LO odometry_;
    lo::pipeline::LOConfig config_;

    std::string odom_frame_{"odom"};
    std::string base_frame_{};

    std::string output_file_path_; 

    int frame_counter_ = 0;
    double total_wall_time_ = 0.0;
};

}  
