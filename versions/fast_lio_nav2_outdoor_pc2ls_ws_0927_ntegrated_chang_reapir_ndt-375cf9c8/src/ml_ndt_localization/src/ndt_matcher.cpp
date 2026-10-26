#include "ml_ndt_localization/ndt_matcher.hpp"
#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>
#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/point_tests.h>
#include <chrono>
#include <algorithm>
#include <cmath>

namespace ml_ndt_loc
{
namespace {
using Clock = std::chrono::steady_clock;
double Elapsed(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
}
struct NdtMatcher::Impl {
    Config config;
    pclomp::MultiGridNormalDistributionsTransform<PointType, PointType> ndt;
    PointCloud::Ptr target;
    // 仅用于独立的全扫描覆盖率检查，不参与 NDT 优化与对应点选择。
    pcl::KdTreeFLANN<PointType> coverage_tree;
    explicit Impl(const Config& c) : config(c) {
        pclomp::NdtParams params;
        params.num_threads = c.threads;
        params.resolution = c.resolution;
        params.step_size = c.step_size;
        params.trans_epsilon = c.transformation_epsilon;
        params.max_iterations = c.max_iterations;
        params.regularization_scale_factor = 0.0;
        params.search_method = pclomp::KDTREE;
        params.use_line_search = false;
        ndt.setParams(params);
    }
};
NdtMatcher::NdtMatcher(const Config& config) : impl_(std::make_unique<Impl>(config)) {}
NdtMatcher::~NdtMatcher() = default;
NdtMatcher::Result NdtMatcher::Align(const PointCloud::Ptr& source,
    const PointCloud::Ptr& target, const Eigen::Matrix4d& initial)
{
    Result result;
    if (impl_->target != target) {
        const auto start = Clock::now();
        impl_->ndt.setInputTarget(target);
        impl_->coverage_tree.setInputCloud(target);
        impl_->target = target;
        result.target_build_ms = Elapsed(start);
    }
    PointCloud aligned;
    const auto start = Clock::now();
    impl_->ndt.align(aligned, initial.cast<float>(), source);
    result.registration_ms = Elapsed(start);
    result.T_map_base = impl_->ndt.getFinalTransformation().cast<double>();
    result.iterations = impl_->ndt.getFinalNumIteration();
    result.likelihood = impl_->ndt.getNearestVoxelTransformationLikelihood();
    result.transformation_probability = impl_->ndt.getTransformationProbability();
    result.converged = result.iterations > 0 && result.iterations < impl_->config.max_iterations &&
                       result.T_map_base.allFinite() && std::isfinite(result.likelihood);
    // NVTL 的分母不包含无邻域的点，因此另外以完整扫描计算 1 m 地图覆盖率。
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    size_t inliers = 0;
    for (const auto& point : aligned) {
        if (pcl::isFinite(point) && impl_->coverage_tree.nearestKSearch(point, 1, indices, distances) > 0 &&
            distances[0] < 1.0f) ++inliers;
    }
    result.inlier_fraction = source->empty() ? 0.0 : double(inliers) / source->size();
    if (result.converged && impl_->config.estimate_covariance) {
        const auto covariance_start = Clock::now();
        // 不调用 getResult()/getHessian()：上游部分提前退出路径不会刷新 Hessian。
        pclomp::NdtResult center{};
        center.pose = result.T_map_base.cast<float>();
        const auto poses = pclomp::propose_poses_to_search(center,
            {0., 0., .5, -.5, 1., -1.}, {.5, -.5, 0., 0., 0., 0.});
        std::vector<Eigen::Vector2d> positions{result.T_map_base.block<2,1>(0,3)};
        // 与偏移点使用同一评分接口，避免优化循环末值与重评分定义差异。
        std::vector<double> scores{impl_->ndt.calculateNearestVoxelTransformationLikelihood(aligned)};
        PointCloud shifted;
        for (const auto& pose : poses) {
            pcl::transformPointCloud(*source, shifted, pose);
            positions.push_back(pose.block<2,1>(0,3).cast<double>());
            scores.push_back(impl_->ndt.calculateNearestVoxelTransformationLikelihood(shifted));
        }
        result.covariance_valid = std::all_of(scores.begin(), scores.end(),
            [](double score) { return std::isfinite(score); });
        if (result.covariance_valid) {
            const auto weights = pclomp::calc_weight_vec(scores, impl_->config.covariance_temperature);
            result.covariance_xy = pclomp::calculate_weighted_mean_and_cov(positions, weights).second;
            result.covariance_valid = result.covariance_xy.allFinite();
        }
        result.covariance_ms = Elapsed(covariance_start);
    }
    return result;
}
} // namespace ml_ndt_loc
