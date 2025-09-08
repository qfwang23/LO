#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>

#include "VoxelHashMap.hpp"

namespace lo {
    Sophus::SE3d RegisterFrame(
        const std::vector<Eigen::Vector3d> &frame,
        const VoxelHashMap &voxel_map,
        Sophus::SE3d initial_guess,
        double kernel,
        bool use_robust_loss);
}
