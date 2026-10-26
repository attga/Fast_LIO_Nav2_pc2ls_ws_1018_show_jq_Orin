#pragma once
// Coordinates map selection, registration quality and tracking health.
// Input: body scan and T_map_base_init = T_map_odom * T_odom_base(scan time).
// Output: absolute T_map_base measurement. Never owns map->odom or filters poses.

#include <Eigen/Dense>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ml_ndt_localization/submap_cache.hpp"
#include "ml_ndt_localization/ndt_matcher.hpp"

namespace ml_ndt_loc
{

class Tracker
{
public:
    struct Config
    {
        NdtMatcher::Config ndt;
        double likelihood_threshold = 2.3;           // NVTL 下限，不是 ICP 距离评分。
        double inlier_fraction_threshold = 0.4;      // 完整扫描的最近邻 <1 m 占比。
        double max_translation_jump = 2.0;           // 预测与配准的三维平移差，m。
        double max_yaw_jump = 15.0;                  // 预测与配准的航向差，度。
        double submap_size = 200.0;
        double submap_update_margin = 80.0;
        double voxel_size = 0.4;                    // 扫描与常驻地图共用降采样尺度，m。
        bool pause_on_lost = false;
        int max_consecutive_failures = 30;
    };

    // One attempt owns its pose, quality metrics and rejection reason.
    // accepted alone controls whether the result may be published as a measurement.
    struct RegistrationResult
    {
        Eigen::Matrix4d T_map_base = Eigen::Matrix4d::Identity();
        bool accepted = false;
        bool converged = false;
        double likelihood = std::numeric_limits<double>::quiet_NaN();
        double inlier_fraction = std::numeric_limits<double>::quiet_NaN();
        double correction_m = 0.0;
        double correction_yaw_deg = 0.0;
        double transformation_probability = std::numeric_limits<double>::quiet_NaN();
        int iterations = 0;
        double target_build_ms = 0.0;
        Eigen::Matrix2d covariance_xy = Eigen::Matrix2d::Zero();
        bool covariance_valid = true;
        double covariance_ms = 0.0;
        double registration_ms = 0.0;
        double submap_ms = 0.0;
        bool submap_updated = false;
        size_t submap_points = 0;
        double window_center_x = 0.0, window_center_y = 0.0;
        std::string rejection_reason;
    };

    struct Status
    {
        RegistrationResult last_result;
        int fail_count = 0;
        bool lost = false;
    };

    Tracker(const std::string& map_dir,
            const Config& cfg);

    // 调用者查询扫描时刻的 TF 链，构造 T_map_base_init。
    // Tracker 只接收 body 系扫描与 map 系初值，不订阅或同步里程计。
    RegistrationResult TrackOnce(const pcl::PointCloud<PointType>::Ptr& scan_body,
                                 const Eigen::Matrix4d& T_map_base_init);

    pcl::PointCloud<PointType>::ConstPtr GlobalMap() const;

    Status GetStatus();

    bool IsLost();

    // LOST 表示质量失效;pause_on_lost 决定是否暂停等待外部 Reset。
    // 持续尝试模式下,只有通过质量门限的结果才自动清除 LOST。
    void MarkLost();

    // Manager 调 /ml_localization/start_tracking 服务:清零失效计数与 LOST,恢复配准
    void Reset();

private:
    // Caller holds mtx_. Only completed attempts update the consecutive count.
    RegistrationResult RecordResult(const RegistrationResult& result);

    RegistrationResult RecordFailure(const std::string& reason);

    void EvaluateQuality(RegistrationResult& result) const;

    static double YawOf(const Eigen::Matrix4d& T);

    Config cfg_;
    std::unique_ptr<SubmapCache> submap_cache_;
    std::unique_ptr<NdtMatcher> matcher_;

    std::mutex mtx_;
    int fail_count_ = 0;
    bool lost_ = false;
    RegistrationResult last_result_;
};

} // namespace ml_ndt_loc
