#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <memory>
#include <regex>
#include <sophus/se3.hpp>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace tf2 {

inline geometry_msgs::Transform sophusToTransform(const Sophus::SE3d &T) {
    geometry_msgs::Transform t;
    t.translation.x = T.translation().x();
    t.translation.y = T.translation().y();
    t.translation.z = T.translation().z();

    Eigen::Quaterniond q(T.so3().unit_quaternion());
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
    t.rotation.w = q.w();

    return t;
}

inline geometry_msgs::Pose sophusToPose(const Sophus::SE3d &T) {
    geometry_msgs::Pose t;
    t.position.x = T.translation().x();
    t.position.y = T.translation().y();
    t.position.z = T.translation().z();

    Eigen::Quaterniond q(T.so3().unit_quaternion());
    t.orientation.x = q.x();
    t.orientation.y = q.y();
    t.orientation.z = q.z();
    t.orientation.w = q.w();

    return t;
}

inline Sophus::SE3d transformToSophus(const geometry_msgs::TransformStamped &transform) {
    const auto &t = transform.transform;
    return Sophus::SE3d(
        Sophus::SE3d::QuaternionType(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z),
        Sophus::SE3d::Point(t.translation.x, t.translation.y, t.translation.z));
}
} 

namespace lo_ros::utils {
using PointCloud2 = sensor_msgs::PointCloud2;
using PointField = sensor_msgs::PointField;
using Header = std_msgs::Header;

inline std::string FixFrameId(const std::string &frame_id) {
    return std::regex_replace(frame_id, std::regex("^/"), "");
}

inline auto GetTimestampField(const PointCloud2::ConstPtr msg) {
    PointField timestamp_field;
    for (const auto &field : msg->fields) {
        if ((field.name == "t" || field.name == "timestamp" || field.name == "time")) {
            timestamp_field = field;
        }
    }
    if (!timestamp_field.count) {
        throw std::runtime_error("Field 't', 'timestamp', or 'time'  does not exist");
    }
    return timestamp_field;
}

inline auto NormalizeTimestamps(const std::vector<double> &timestamps) {
    const auto [min_it, max_it] = std::minmax_element(timestamps.cbegin(), timestamps.cend());
    const double min_timestamp = *min_it;
    const double max_timestamp = *max_it;

    std::vector<double> timestamps_normalized(timestamps.size());
    std::transform(timestamps.cbegin(), timestamps.cend(), timestamps_normalized.begin(),
                   [&](const auto &timestamp) {
                       return (timestamp - min_timestamp) / (max_timestamp - min_timestamp);
                   });
    return timestamps_normalized;
}

inline auto ExtractTimestampsFromMsg(const PointCloud2::ConstPtr msg, const PointField &field) {
    auto extract_timestamps =
        [&msg]<typename T>(sensor_msgs::PointCloud2ConstIterator<T> &&it) -> std::vector<double> {
        const size_t n_points = msg->height * msg->width;
        std::vector<double> timestamps;
        timestamps.reserve(n_points);
        for (size_t i = 0; i < n_points; ++i, ++it) {
            timestamps.emplace_back(static_cast<double>(*it));
        }
        return NormalizeTimestamps(timestamps);
    };

    auto timestamp_field = GetTimestampField(msg);

    using sensor_msgs::PointCloud2ConstIterator;
    if (timestamp_field.datatype == PointField::UINT32) {
        return extract_timestamps(PointCloud2ConstIterator<uint32_t>(*msg, timestamp_field.name));
    } else if (timestamp_field.datatype == PointField::FLOAT32) {
        return extract_timestamps(PointCloud2ConstIterator<float>(*msg, timestamp_field.name));
    } else if (timestamp_field.datatype == PointField::FLOAT64) {
        return extract_timestamps(PointCloud2ConstIterator<double>(*msg, timestamp_field.name));
    }

    throw std::runtime_error("timestamp field type not supported");
}

inline std::unique_ptr<PointCloud2> CreatePointCloud2Msg(const size_t n_points,
                                                         const Header &header,
                                                         bool timestamp = false) {
    auto cloud_msg = std::make_unique<PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
    cloud_msg->header = header;
    cloud_msg->header.frame_id = FixFrameId(cloud_msg->header.frame_id);
    cloud_msg->fields.clear();
    int offset = 0;
    offset = addPointField(*cloud_msg, "x", 1, PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "y", 1, PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "z", 1, PointField::FLOAT32, offset);
    offset += sizeOfPointField(PointField::FLOAT32);
    if (timestamp) {
        offset = addPointField(*cloud_msg, "time", 1, PointField::FLOAT64, offset);
        offset += sizeOfPointField(PointField::FLOAT64);
    }

    cloud_msg->point_step = offset;
    cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
    cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);
    modifier.resize(n_points);
    return cloud_msg;
}

inline void FillPointCloud2XYZ(const std::vector<Eigen::Vector3d> &points, PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");
    for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z) {
        const Eigen::Vector3d &point = points[i];
        *msg_x = point.x();
        *msg_y = point.y();
        *msg_z = point.z();
    }
}

inline void FillPointCloud2Timestamp(const std::vector<double> &timestamps, PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<double> msg_t(msg, "time");
    for (size_t i = 0; i < timestamps.size(); i++, ++msg_t) *msg_t = timestamps[i];
}

inline std::vector<double> GetTimestamps(const PointCloud2::ConstPtr msg) {
    auto timestamp_field = GetTimestampField(msg);

    std::vector<double> timestamps = ExtractTimestampsFromMsg(msg, timestamp_field);

    return timestamps;
}

inline std::vector<Eigen::Vector3d> PointCloud2ToEigen(const PointCloud2::ConstPtr msg) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(msg->height * msg->width);
    sensor_msgs::PointCloud2ConstIterator<float> msg_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> msg_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> msg_z(*msg, "z");
    for (size_t i = 0; i < msg->height * msg->width; ++i, ++msg_x, ++msg_y, ++msg_z) {
        points.emplace_back(*msg_x, *msg_y, *msg_z);
    }
    return points;
}

inline std::unique_ptr<PointCloud2> EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                                       const Header &header) {
    auto msg = CreatePointCloud2Msg(points.size(), header);
    FillPointCloud2XYZ(points, *msg);
    return msg;
}

inline std::unique_ptr<PointCloud2> EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                                       const Sophus::SE3d &T,
                                                       const Header &header) {
    std::vector<Eigen::Vector3d> points_t;
    points_t.resize(points.size());
    std::transform(points.cbegin(), points.cend(), points_t.begin(),
                   [&](const auto &point) { return T * point; });
    return EigenToPointCloud2(points_t, header);
}

inline std::unique_ptr<PointCloud2> EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                                                       const std::vector<double> &timestamps,
                                                       const Header &header) {
    auto msg = CreatePointCloud2Msg(points.size(), header, true);
    FillPointCloud2XYZ(points, *msg);
    FillPointCloud2Timestamp(timestamps, *msg);
    return msg;
}
}  
