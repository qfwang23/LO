#include "LO.hpp"

#include <Eigen/Core>
#include <tbb/parallel_for.h>
#include <tsl/robin_map.h>

#include <tuple>
#include <vector>

#include "LO/core/Registration.hpp"
#include "LO/core/VoxelHashMap.hpp"
#include "LO/core/GroundEstimator.hpp"
#include "LO/core/MyGroundRemover.hpp"

namespace {
using Voxel = Eigen::Vector3i;

struct VoxelHash {
    size_t operator()(const Voxel& voxel) const {
        const uint32_t* vec = reinterpret_cast<const uint32_t*>(voxel.data());
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
};
}  // namespace

namespace lo {

std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d>& frame,
                                             double voxel_size) {
    tsl::robin_map<Voxel, Eigen::Vector3d, VoxelHash> grid;
    grid.reserve(frame.size());

    for (const auto& point : frame) {
        const auto voxel = Voxel((point / voxel_size).cast<int>());
        grid.insert({voxel, point});
    }

    std::vector<Eigen::Vector3d> frame_dowsampled;
    frame_dowsampled.reserve(grid.size());

    for (const auto& [voxel, point] : grid) {
        (void)voxel;
        frame_dowsampled.push_back(point);
    }

    return frame_dowsampled;
}

double AdaptiveThreshold::ComputeThreshold() {
    double model_error = model_deviation_.translation().norm();

    if (model_error > min_motion_th_) {
        model_error_sse2_ += model_error * model_error;
        num_samples_++;
    }

    if (num_samples_ == 0) {
        return initial_threshold_;
    }

    return std::sqrt(model_error_sse2_ / num_samples_);
}

}  // namespace lo

namespace lo::pipeline {

LO::Vector3dVectorTuple LO::RegisterFrame(const std::vector<Eigen::Vector3d>& frame) {
    const auto& cropped_frame = Preprocess(frame, config_.max_range, config_.min_range);
    const auto& [source, frame_downsample] = Voxelize(cropped_frame);

    has_moved = HasMoved();

    const double sigma_frame = GetAdaptive_frame();
    const double sigma_local = GetAdaptive_local();

    auto prediction  = LO::GetPredictionModel();
    auto last_pose   = !poses_.empty() ? poses_.back() : Sophus::SE3d();
    auto initial_guess = last_pose * prediction;

    const size_t N = poses_.size();

    if (N >= 2) {
        prediction = lo::RegisterFrame(source,
                                       last_frame,
                                       initial_guess,
                                       sigma_frame,
                                       false);
    }

    const Sophus::SE3d new_pose = lo::RegisterFrame(source,
                                                    local_map_,
                                                    prediction,
                                                    sigma_local,
                                                    true);

    const auto model_deviation_frame = initial_guess.inverse() * prediction;
    const auto model_deviation_local = prediction.inverse() * new_pose;

    adaptive_frame.UpdateModelDeviation(model_deviation_frame);
    adaptive_local.UpdateModelDeviation(model_deviation_local);

    local_map_.Update(source, new_pose, false);
    global_map_.Update(lo::VoxelDownsample(source, 2.0), new_pose, true);
    last_frame.Update(frame_downsample, new_pose, false, true);

    poses_.push_back(new_pose);
    return {frame, source};
}

LO::Vector3dVectorTuple LO::Voxelize(const std::vector<Eigen::Vector3d>& frame) const {
    const auto voxel_size       = config_.voxel_size;
    const auto frame_downsample = lo::VoxelDownsample(frame, voxel_size * 0.5);
    const auto source           = lo::VoxelDownsample(frame_downsample, voxel_size * 0.75);
    return {source, frame_downsample};
}

double LO::GetAdaptive_frame() {
    if (!has_moved) {
        return config_.initial_threshold;
    }
    return adaptive_frame.ComputeThreshold();
}

double LO::GetAdaptive_local() {
    if (!has_moved) {
        return config_.initial_threshold;
    }
    return adaptive_local.ComputeThreshold();
}

Sophus::SE3d LO::GetPredictionModel() const {
    Sophus::SE3d pred = Sophus::SE3d();
    const size_t N = poses_.size();
    if (N < 2) return pred;
    return poses_[N - 2].inverse() * poses_[N - 1];
}

bool LO::HasMoved() {
    if (poses_.empty()) return false;
    const double motion = (poses_.front().inverse() * poses_.back()).translation().norm();
    return motion > 5.0 * config_.min_motion_th;
}

std::vector<Eigen::Vector3d> LO::Preprocess(const std::vector<Eigen::Vector3d>& frame,
                                            double max_range,
                                            double min_range) {
    std::vector<Eigen::Vector3d> inliers;
    std::copy_if(frame.cbegin(), frame.cend(), std::back_inserter(inliers), [&](const auto& pt) {
        const double norm = pt.norm();
        return norm < max_range && norm > min_range;
    });
    return inliers;
}

}  // namespace lo::pipeline
