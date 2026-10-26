#include "ml_icp_localization/scan_matcher.hpp"
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/impl/fast_gicp_impl.hpp>
#include <pcl/registration/gicp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <chrono>
#include <stdexcept>

namespace ml_icp_loc
{
namespace
{
// Use FastGICP's own covariance algorithm and regularization, once at startup.
// Global map neighborhoods remain consistent when a window boundary moves.
class MapCovarianceEstimator : public fast_gicp::FastGICP<PointType, PointType>
{
public:
    using Covariances = std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>;
    Covariances Compute(const pcl::PointCloud<PointType>::ConstPtr& cloud, int threads)
    {
        setNumThreads(threads);
        setCorrespondenceRandomness(20);
        setInputTarget(cloud);
        if (!calculate_covariances<PointType>(cloud, *search_target_, target_covs_))
            throw std::runtime_error("failed to prepare map covariances");
        return std::move(target_covs_);
    }
};
} // namespace

struct ScanMatcher::Impl
{
    struct Config {
        std::string registration_backend;
        int registration_threads;
        double correspondence_distance;
    } cfg_;
    bool cache_target_;
    MapCovarianceEstimator::Covariances map_covariances_;
    std::unique_ptr<pcl::Registration<PointType, PointType>> registration_;
    PointCloud::Ptr target_;
    pcl::KdTreeFLANN<PointType> target_tree_;

    Impl(const std::string& backend, int threads, double distance, bool cache_target,
         const PointCloud::ConstPtr& map)
        : cfg_{backend, threads, distance}, cache_target_(cache_target)
    {
        if (backend != "pcl_gicp" && backend != "fast_gicp")
            throw std::invalid_argument("registration_backend must be pcl_gicp or fast_gicp");
        if (threads < 1) throw std::invalid_argument("registration_threads must be >= 1");
        if (cache_target_) {
            registration_ = CreateRegistration();
            if (backend == "fast_gicp" && map) {
                MapCovarianceEstimator estimator;
                map_covariances_ = estimator.Compute(map, threads);
            }
        }
    }

    std::unique_ptr<pcl::Registration<PointType, PointType>> CreateRegistration() const
    {
        std::unique_ptr<pcl::Registration<PointType, PointType>> registration;
        if (cfg_.registration_backend == "fast_gicp") {
            auto fast = std::make_unique<fast_gicp::FastGICP<PointType, PointType>>();
            fast->setNumThreads(cfg_.registration_threads);
            fast->setCorrespondenceRandomness(20);
            // FastGICP uses its own optimizer and convergence criteria; it does
            // not implement PCL ICP's reciprocal-correspondence option.
            registration = std::move(fast);
        } else {
            auto pcl_gicp = std::make_unique<pcl::GeneralizedIterativeClosestPoint<PointType, PointType>>();
            pcl_gicp->setUseReciprocalCorrespondences(true);
            registration = std::move(pcl_gicp);
        }
        auto& icp = *registration;
        icp.setMaxCorrespondenceDistance(cfg_.correspondence_distance);
        icp.setMaximumIterations(50);
        // FastGICP's native translation stopping threshold is 5e-4 m.
        // Unlike PCL GICP, reaching the iteration limit does not imply convergence.
        icp.setTransformationEpsilon(
            cfg_.registration_backend == "fast_gicp" ? 5e-4 : 1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);
        return registration;
    }

    // 配准后 aligned 点在 target 上最近邻 <dist 的占比(kd-tree)
    static double ComputeInlierFraction(const pcl::PointCloud<PointType>::Ptr& aligned,
                                        const pcl::PointCloud<PointType>::Ptr& target,
                                        double dist, pcl::KdTreeFLANN<PointType>* cached = nullptr)
    {
        if (aligned->empty() || target->empty()) return 0.0;
        pcl::KdTreeFLANN<PointType> kd;
        if (!cached) kd.setInputCloud(target);
        auto& search = cached ? *cached : kd;
        int inliers = 0;
        std::vector<int> idx(1);
        std::vector<float> sqd(1);
        for (const auto& p : aligned->points) {
            if (search.nearestKSearch(p, 1, idx, sqd) > 0 && sqd[0] < dist * dist)
                inliers++;
        }
        return double(inliers) / aligned->size();
    }

};

ScanMatcher::ScanMatcher(const std::string& backend, int threads, double distance,
                         bool cache_target, const PointCloud::ConstPtr& map)
    : impl_(std::make_unique<Impl>(backend, threads, distance, cache_target, map)) {}
ScanMatcher::~ScanMatcher() = default;

ScanMatcher::Result ScanMatcher::Align(const PointCloud::Ptr& source,
                                       const PointCloud::Ptr& target,
                                       const std::vector<size_t>& map_indices)
{
    auto& state = *impl_;
    // Keyframe baseline intentionally creates a fresh registration each attempt.
    auto baseline = state.cache_target_ ? nullptr : state.CreateRegistration();
    auto& icp = state.cache_target_ ? *state.registration_ : *baseline;
    icp.setInputSource(source);
    if (!state.cache_target_ || state.target_ != target) {
        icp.setInputTarget(target);
        if (state.cache_target_ && !state.map_covariances_.empty()) {
            MapCovarianceEstimator::Covariances covariances;
            covariances.reserve(target->size());
            for (size_t index : map_indices) covariances.push_back(state.map_covariances_[index]);
            static_cast<fast_gicp::FastGICP<PointType, PointType>&>(icp).setTargetCovariances(covariances);
        }
        if (state.cache_target_) {
            state.target_ = target;
            state.target_tree_.setInputCloud(target);
        }
    }
    auto aligned = pcl::make_shared<PointCloud>();
    Result result;
    const auto start = std::chrono::steady_clock::now();
    icp.align(*aligned);
    result.registration_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    result.T_target_source = icp.getFinalTransformation().cast<double>();
    result.converged = icp.hasConverged();
    // PCL compares squared nearest-neighbor distances to this limit.
    result.fitness = icp.getFitnessScore(state.cfg_.correspondence_distance * state.cfg_.correspondence_distance);
    result.inlier_fraction = Impl::ComputeInlierFraction(
        aligned, target, 1.0, state.cache_target_ ? &state.target_tree_ : nullptr);
    return result;
}
} // namespace ml_icp_loc
