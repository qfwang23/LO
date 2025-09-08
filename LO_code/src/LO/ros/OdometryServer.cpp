#include <Eigen/Core>
#include <memory>
#include <utility>
#include <vector>

#include <fstream> 

#include "OdometryServer.hpp"
#include "Utils.hpp"

#include "LO/pipeline/LO.hpp"

// ROS 1 headers
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <fstream>
#include <iomanip>  

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


namespace lo_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), tf2_listener_(tf2_ros::TransformListener(tf2_buffer_)) {
    pnh_.param("base_frame", base_frame_, base_frame_);
    pnh_.param("odom_frame", odom_frame_, odom_frame_);
    pnh_.param("publish_odom_tf", publish_odom_tf_, false);
    pnh_.param("visualize", publish_debug_clouds_, publish_debug_clouds_);
    pnh_.param("max_range", config_.max_range, config_.max_range);
    pnh_.param("min_range", config_.min_range, config_.min_range);
    pnh_.param("deskew", config_.deskew, config_.deskew);
    pnh_.param("voxel_size", config_.voxel_size, config_.max_range / 100.0);
    pnh_.param("max_points_per_voxel", config_.max_points_per_voxel, config_.max_points_per_voxel);
    pnh_.param("initial_threshold", config_.initial_threshold, config_.initial_threshold);
    pnh_.param("min_motion_th", config_.min_motion_th, config_.min_motion_th);
    pnh_.param("output_file_path", output_file_path_, std::string("/home/wqf/Result/lo/123.txt"));
    if (config_.max_range < config_.min_range) {
        ROS_WARN("[WARNING] max_range is smaller than min_range, setting min_range to 0.0");
        config_.min_range = 0.0;
    }

    odometry_ = lo::pipeline::LO(config_);

    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("pointcloud_topic", queue_size_,
                                                              &OdometryServer::RegisterFrame, this);

    odom_publisher_ = pnh_.advertise<nav_msgs::Odometry>("/lo/odometry", queue_size_);
    traj_publisher_ = pnh_.advertise<nav_msgs::Path>("/lo/trajectory", queue_size_);
    if (publish_debug_clouds_) {
        frame_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/lo/frame", queue_size_);
        kpoints_publisher_ =
            pnh_.advertise<sensor_msgs::PointCloud2>("/lo/keypoints", queue_size_);
        map_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/lo/local_map", queue_size_);

        global_map_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/lo/global_map", queue_size_);

    }

    tf2_buffer_.setUsingDedicatedThread(true);
    path_msg_.header.frame_id = odom_frame_;

    ROS_INFO("KISS-ICP ROS 1 Odometry Node Initialized");
}

Sophus::SE3d OdometryServer::LookupTransform(const std::string &target_frame,
                                             const std::string &source_frame) const {
    std::string err_msg;
    if (tf2_buffer_._frameExists(source_frame) &&  //
        tf2_buffer_._frameExists(target_frame) &&  //
        tf2_buffer_.canTransform(target_frame, source_frame, ros::Time(0), &err_msg)) {
        try {
            auto tf = tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
            return tf2::transformToSophus(tf);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }
    ROS_WARN("Failed to find tf between %s and %s. Reason=%s", target_frame.c_str(),
             source_frame.c_str(), err_msg.c_str());
    return {};
}

void OdometryServer::RegisterFrame(const sensor_msgs::PointCloud2::ConstPtr &msg) {

    frame_counter_++;

    std::cout << "\n========== Frame " << frame_counter_ << " Begin ==========" << std::endl;

    auto wall_start = std::chrono::steady_clock::now();


    const auto cloud_frame_id = msg->header.frame_id;
    const auto points = PointCloud2ToEigen(msg);

    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == cloud_frame_id);

    const auto &[frame, keypoints] = odometry_.RegisterFrame(points);

    auto wall_end = std::chrono::steady_clock::now();

    double wall_ms = std::chrono::duration<double, std::milli>(wall_end - wall_start).count();

    total_wall_time_ += wall_ms;

    std::cout << "Frame " << frame_counter_ << " finished." << std::endl;
    std::cout << "Wall time: " << wall_ms << " ms" << std::endl;
    std::cout << "========== Frame " << frame_counter_ << " End ==========\n" << std::endl;

    const Sophus::SE3d kiss_pose = odometry_.poses().back();

    const auto pose = [&]() -> Sophus::SE3d {
        if (egocentric_estimation) return kiss_pose;
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        return cloud2base * kiss_pose * cloud2base.inverse();
    }();

    PublishOdometry(pose, msg->header.stamp, cloud_frame_id);

    SavePoseToTUMFormat(pose, msg->header.stamp, output_file_path_);
    

    if (publish_debug_clouds_) {
        PublishClouds(frame, keypoints, msg->header.stamp, cloud_frame_id);
    }
}

void OdometryServer::PublishOdometry(const Sophus::SE3d &pose,
                                     const ros::Time &stamp,
                                     const std::string &cloud_frame_id) {

    if (publish_odom_tf_) {
        geometry_msgs::TransformStamped transform_msg;
        transform_msg.header.stamp = stamp;
        transform_msg.header.frame_id = odom_frame_;
        transform_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;
        transform_msg.transform = tf2::sophusToTransform(pose);
        tf_broadcaster_.sendTransform(transform_msg);
    }

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = odom_frame_;
    pose_msg.pose = tf2::sophusToPose(pose);
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_.publish(path_msg_);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.pose.pose = tf2::sophusToPose(pose);
    odom_publisher_.publish(odom_msg);
}

void OdometryServer::PublishClouds(const std::vector<Eigen::Vector3d> frame,
                                   const std::vector<Eigen::Vector3d> keypoints,
                                   const ros::Time &stamp,
                                   const std::string &cloud_frame_id) {
    std_msgs::Header odom_header;
    odom_header.stamp = stamp;
    odom_header.frame_id = odom_frame_;

    const auto kiss_map = odometry_.LocalMap();

    const auto global_map = odometry_.GlobalMap();

    if (!publish_odom_tf_) {
        std_msgs::Header cloud_header;
        cloud_header.stamp = stamp;
        cloud_header.frame_id = cloud_frame_id;

        frame_publisher_.publish(*EigenToPointCloud2(frame, cloud_header));
        kpoints_publisher_.publish(*EigenToPointCloud2(keypoints, cloud_header));
        map_publisher_.publish(*EigenToPointCloud2(kiss_map, odom_header));

        global_map_publisher_.publish(*EigenToPointCloud2(global_map, odom_header));

        return;
    }

    const auto cloud2odom = LookupTransform(odom_frame_, cloud_frame_id);
    frame_publisher_.publish(*EigenToPointCloud2(frame, cloud2odom, odom_header));
    kpoints_publisher_.publish(*EigenToPointCloud2(keypoints, cloud2odom, odom_header));

    if (!base_frame_.empty()) {
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        map_publisher_.publish(*EigenToPointCloud2(kiss_map, cloud2base, odom_header));
    } else {
        map_publisher_.publish(*EigenToPointCloud2(kiss_map, odom_header));
    }
}

void OdometryServer::SaveGlobalMapAsPCD(const std::string& filename) const {
    const auto global_map = odometry_.GlobalMap();

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    for (const auto& point : global_map) {
        pcl_cloud->points.emplace_back(point.x(), point.y(), point.z());
    }

    pcl_cloud->width = pcl_cloud->points.size();
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = true;

    if (pcl::io::savePCDFileASCII(filename, *pcl_cloud) == -1) {
        ROS_ERROR("Failed to save global map to PCD file!");
    } else {
        ROS_INFO("Global map successfully saved to %s", filename.c_str());
    }
}



void OdometryServer::SavePoseToTUMFormat(const Sophus::SE3d& pose, const ros::Time& timestamp, const std::string& filename) const {
    std::ofstream file;
    file.open(filename, std::ios::app);  // Open the file in append mode

    // 获取平移向量和旋转四元数
    Eigen::Vector3d translation = pose.translation();
    Eigen::Quaterniond quaternion = pose.unit_quaternion();

    // 写入时间戳、平移、旋转信息
    file << std::fixed << std::setprecision(10) 
         << timestamp.toSec() << " "
         << translation.x() << " " << translation.y() << " " << translation.z() << " "
         << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " << quaternion.w() << "\n";

    file.close();
}

void OdometryServer::PrintStatistics() const {
    std::cout << "\n=========== Summary ===========" << std::endl;
    std::cout << "Processed " << frame_counter_ << " frames." << std::endl;
    if (frame_counter_ > 0) {
        std::cout << "Average Wall time per frame: " << (total_wall_time_ / frame_counter_) << " ms" << std::endl;
    }
}

} 

int main(int argc, char **argv) {
    ros::init(argc, argv, "lo");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    lo_ros::OdometryServer node(nh, nh_private);

    ros::spin();

    node.PrintStatistics(); 


    // 节点终止后保存全局地图到PCD
    // std::string global_map_file = "/home/wqf/DataSet/M2DGR/global_map.pcd";
    // node.SaveGlobalMapAsPCD(global_map_file);

    return 0;
}
