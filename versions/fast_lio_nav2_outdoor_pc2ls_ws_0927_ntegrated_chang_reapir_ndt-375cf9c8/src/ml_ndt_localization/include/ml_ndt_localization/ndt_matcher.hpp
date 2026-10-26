#pragma once
#include "ml_ndt_localization/point_types.hpp"
#include <Eigen/Dense>
#include <limits>
#include <memory>

namespace ml_ndt_loc
{
// 适配固定版本的 Autoware NDT。只持有目标地图结构，不积累机器人位姿。
class NdtMatcher
{
public:
    struct Config {
        int threads = 2;
        double resolution = 2.0;
        double step_size = 0.1;
        double transformation_epsilon = 0.01;
        int max_iterations = 30;
        bool estimate_covariance = true;
        double covariance_temperature = 0.05;
    };
    struct Result {
        Eigen::Matrix4d T_map_base = Eigen::Matrix4d::Identity();
        // 上游没有 hasConverged；按 scan matcher 的迭代上限规则判断。
        bool converged = false;
        int iterations = 0;
        double likelihood = std::numeric_limits<double>::quiet_NaN();
        double transformation_probability = std::numeric_limits<double>::quiet_NaN();
        double inlier_fraction = 0.0;
        double registration_ms = 0.0;
        double target_build_ms = 0.0;
        Eigen::Matrix2d covariance_xy = Eigen::Matrix2d::Zero(); // map 系评分分布方差，m²；未叠加测量底噪。
        bool covariance_valid = true;
        double covariance_ms = 0.0;
    };
    explicit NdtMatcher(const Config& config);
    ~NdtMatcher();
    // source 位于 base，target 位于 map；输入与输出都是绝对 T_map_base。
    Result Align(const PointCloud::Ptr& source, const PointCloud::Ptr& target,
                 const Eigen::Matrix4d& T_map_base_init);
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace ml_ndt_loc
