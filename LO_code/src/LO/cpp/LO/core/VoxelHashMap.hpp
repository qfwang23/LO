#pragma once

#include <tsl/robin_map.h>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <tuple>
#include <vector>

namespace lo {

struct VoxelHashMap {
    using Vector3dVector      = std::vector<Eigen::Vector3d>;
    using Vector3dVectorTuple = std::tuple<Vector3dVector, Vector3dVector>;
    using Voxel               = Eigen::Vector3i;

    struct VoxelBlock {
        std::vector<Eigen::Vector3d> points;
        int num_points_;
        inline void AddPoint(const Eigen::Vector3d& point) {
            if (points.size() < static_cast<size_t>(num_points_)) points.push_back(point);
        }
    };

    struct VoxelHash {
        size_t operator()(const Voxel& voxel) const {
            const uint32_t* vec = reinterpret_cast<const uint32_t*>(voxel.data());
            return ((1 << 20) - 1) &
                   (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
        }
    };

    explicit VoxelHashMap(double voxel_size, double max_distance, int max_points_per_voxel)
        : voxel_size_(voxel_size),
          max_distance_(max_distance),
          max_points_per_voxel_(max_points_per_voxel) {}

    Vector3dVectorTuple GetCorrespondences(const Vector3dVector& points) const;

    inline void Clear() { map_.clear(); }
    inline bool Empty() const { return map_.empty(); }

    void Update(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& origin);
    void Update(const std::vector<Eigen::Vector3d>& points, const Sophus::SE3d& pose, const bool is_global);
    void Update(const std::vector<Eigen::Vector3d>& points, const Sophus::SE3d& pose, const bool is_global, const bool is_last_frame);

    void AddPoints(const std::vector<Eigen::Vector3d>& points);
    void RemovePointsFarFromLocation(const Eigen::Vector3d& origin);
    std::vector<Eigen::Vector3d> Pointcloud() const;

    void Update(const Vector3dVector& points);

    double voxel_size_;
    double max_distance_;
    int max_points_per_voxel_;
    tsl::robin_map<Voxel, VoxelBlock, VoxelHash> map_;
};

}  // namespace lo
