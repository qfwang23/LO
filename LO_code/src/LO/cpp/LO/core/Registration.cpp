#include "Registration.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>


namespace Eigen {
using Matrix6d   = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d   = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

namespace {

inline double square(double x) { return x * x; }

struct ResultTuple {
    ResultTuple() {
        JTJ.setZero();
        JTr.setZero();
    }

    ResultTuple operator+(const ResultTuple& other) {
        this->JTJ += other.JTJ;
        this->JTr += other.JTr;
        return *this;
    }

    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
};

void TransformPointsInPlace(Sophus::SE3d& T, std::vector<Eigen::Vector3d>& points) {
    for (auto& point : points) {
        point = T * point;
    }
}

constexpr int   MAX_NUM_ITERATIONS_   = 500;
constexpr double ESTIMATION_THRESHOLD_ = 0.00001;

std::tuple<Eigen::Matrix6d, Eigen::Vector6d> BuildLinearSystem(
    const std::vector<Eigen::Vector3d>& source,
    const std::vector<Eigen::Vector3d>& target,
    double kernel,
    bool use_robust_loss = false) {
    const auto compute_jacobian_and_residual = [&](size_t i) {
        const Eigen::Vector3d residual = source[i] - target[i];
        Eigen::Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source[i]);
        return std::make_tuple(J_r, residual);
    };

    auto Weight = [&](double residual2) {
        if (use_robust_loss) {
            double delta = kernel;
            if (residual2 < delta * delta) {
                return 1.0;
            } else {
                return delta / (delta + std::sqrt(residual2));
            }
        } else {
            double delta = kernel;
            if (residual2 < delta * delta) {
                return 1.0;
            } else {
                return square(delta) / square(delta + residual2);
            }
        }
    };

    ResultTuple result = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, source.size()),
        ResultTuple(),
        [&](const tbb::blocked_range<size_t>& r, ResultTuple local_result) {
            auto& [JTJ_private, JTr_private] = local_result;
            for (size_t i = r.begin(); i < r.end(); ++i) {
                const auto& [J_r, residual] = compute_jacobian_and_residual(i);
                const double w = Weight(residual.squaredNorm());
                JTJ_private.noalias() += J_r.transpose() * w * J_r;
                JTr_private.noalias() += J_r.transpose() * w * residual;
            }
            return local_result;
        },
        [](ResultTuple a, const ResultTuple& b) -> ResultTuple {
            return a + b;
        });

    return std::make_tuple(result.JTJ, result.JTr);
}

}  // namespace

namespace lo {

Sophus::SE3d RegisterFrame(const std::vector<Eigen::Vector3d>& frame,
                           const VoxelHashMap& voxel_map,
                           Sophus::SE3d initial_guess,
                           double kernel,
                           bool use_robust_loss /*= false*/) {
    if (voxel_map.Empty()) return initial_guess;

    std::vector<Eigen::Vector3d> source = frame;
    TransformPointsInPlace(initial_guess, source);

    Sophus::SE3d T_icp = Sophus::SE3d();
    for (int j = 0; j < MAX_NUM_ITERATIONS_; ++j) {
        auto corr       = voxel_map.GetCorrespondences(source);
        const auto& src = std::get<0>(corr);
        const auto& tgt = std::get<1>(corr);

        auto [JTJ, JTr] = BuildLinearSystem(src, tgt, kernel, use_robust_loss);

        JTJ.diagonal().array() += 1e-6;

        Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);

        auto predicted_gain = [&](const Eigen::Vector6d& step) -> double {
            return -JTr.dot(step) - 0.5 * step.dot(JTJ * step);
        };
        const double dz_gate  = 0.03;
        const double dz_limit = 0.05;
        if (std::abs(dx(2)) > dz_gate) {
            Eigen::Vector6d dx_clamp = dx;
            dx_clamp(2)              = std::copysign(dz_limit, dx(2));
            Eigen::Vector6d dx_zero  = dx;
            dx_zero(2)               = 0.0;

            double g_full  = predicted_gain(dx);
            double g_clamp = predicted_gain(dx_clamp);
            double g_zero  = predicted_gain(dx_zero);

            if (g_clamp >= g_full && g_clamp >= g_zero) {
                dx = dx_clamp;
            } else if (g_zero >= g_full && g_zero >= g_clamp) {
                dx = dx_zero;
            }
        }

        Sophus::SE3d estimation = Sophus::SE3d::exp(dx);
        TransformPointsInPlace(estimation, source);
        T_icp = estimation * T_icp;

        if (dx.norm() < ESTIMATION_THRESHOLD_) break;
    }

    Sophus::SE3d   T_out = T_icp * initial_guess;
    Eigen::Vector3d t0    = initial_guess.translation();
    Eigen::Vector3d t     = T_out.translation();

    const double dz_frame_limit = 0.08;
    double       dz             = t.z() - t0.z();
    if (std::abs(dz) > dz_frame_limit) {
        t.z() = t0.z() + std::copysign(dz_frame_limit, dz);
        T_out = Sophus::SE3d(T_out.so3(), t);
    }

    return T_out;
}

}  // namespace lo
