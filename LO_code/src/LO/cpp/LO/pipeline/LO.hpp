#pragma once

#include <Eigen/Core>
#include <tuple>
#include <vector>

#include "LO/core/VoxelHashMap.hpp"

namespace Sophus {
class SE3d;
}

namespace lo {

std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d>& frame,
                                             double voxel_size);

std::vector<Eigen::Vector3d> RemoveGroundPointsAdvanced(const std::vector<Eigen::Vector3d>& points,
                                                        double grid_resolution,
                                                        double delta_h1,
                                                        double delta_h2,
                                                        double normal_angle_thresh_deg,
                                                        double pca_radius);

struct AdaptiveThreshold {
    explicit AdaptiveThreshold(double initial_threshold, double min_motion_th, double max_range)
        : initial_threshold_(initial_threshold),
          min_motion_th_(min_motion_th),
          max_range_(max_range) {}

    inline void UpdateModelDeviation(const Sophus::SE3d& current_deviation) {
        model_deviation_ = current_deviation;
    }

    double ComputeThreshold();

private:
    double initial_threshold_;
    double min_motion_th_;
    double max_range_;

    double     model_error_sse2_ = 0;
    int        num_samples_      = 0;
    Sophus::SE3d model_deviation_ = Sophus::SE3d();
};

}  // namespace lo

namespace lo::pipeline {

struct LOConfig {
    double voxel_size           = 1.0;
    double max_range            = 100.0;
    double min_range            = 5.0;
    int    max_points_per_voxel = 20;

    double min_motion_th      = 0.1;
    double initial_threshold  = 2.0;

    bool deskew = false;
};

class LO {
public:
    using Vector3dVector      = std::vector<Eigen::Vector3d>;
    using Vector3dVectorTuple = std::tuple<Vector3dVector, Vector3dVector>;

public:
    explicit LO(const LOConfig& config)
        : config_(config),
          local_map_(config.voxel_size, config.max_range, config.max_points_per_voxel),
          global_map_(config.voxel_size, config.max_range, config.max_points_per_voxel),
          last_frame(config.voxel_size, config.max_range, config.max_points_per_voxel),
          adaptive_frame(config.initial_threshold, config.min_motion_th, config.max_range),
          adaptive_local(config.initial_threshold, config.min_motion_th, config.max_range) {}

    LO() : LO(LOConfig{}) {}

public:
    Vector3dVectorTuple RegisterFrame(const std::vector<Eigen::Vector3d>& frame);
    Vector3dVectorTuple Voxelize(const std::vector<Eigen::Vector3d>& frame) const;

    double GetAdaptive_frame();
    double GetAdaptive_local();

    Sophus::SE3d GetPredictionModel() const;
    bool         HasMoved();

    std::vector<Eigen::Vector3d> Dedistortion(const std::vector<Eigen::Vector3d>& frame);

    std::vector<Eigen::Vector3d> Preprocess(const std::vector<Eigen::Vector3d>& frame,
                                            double max_range,
                                            double min_range);

public:
    std::vector<Eigen::Vector3d> LocalMap() const { return local_map_.Pointcloud(); }
    std::vector<Eigen::Vector3d> GlobalMap() const { return global_map_.Pointcloud(); }
    std::vector<Sophus::SE3d>    poses() const { return poses_; }

private:
    std::vector<Sophus::SE3d> poses_;
    LOConfig                  config_;
    VoxelHashMap              local_map_;
    VoxelHashMap              global_map_;
    VoxelHashMap              last_frame;
    AdaptiveThreshold         adaptive_frame;
    AdaptiveThreshold         adaptive_local;
    bool                      has_moved = false;
};

}  // namespace lo::pipeline
