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

#include "ml_icp_localization/map_loader.hpp"
#include "ml_icp_localization/submap_cache.hpp"
#include "ml_icp_localization/scan_matcher.hpp"

namespace ml_icp_loc
{

class Tracker
{
public:
    struct Config
    {
        std::string registration_backend = "pcl_gicp"; // pcl_gicp / fast_gicp
        int registration_threads = 2;                 // FastGICP CPU threads
        double correspondence_distance = 3.0;        // GICP 对应距离(m)
        double fitness_threshold = 0.3;              // 对应距离内平均平方距离闸门(m²)
        double inlier_fraction_threshold = 0.4;      // inlier 比例闸门(最近邻<1m 占比)
        double max_translation_jump = 2.0;           // 预测-配准一致性:位置(m)
        double max_yaw_jump = 15.0;                  // 预测-配准一致性:yaw(deg)
        std::string submap_strategy = "keyframes"; // keyframes (baseline) / window
        double submap_size = 200.0;                 // XY window side length (m)
        double submap_update_margin = 80.0;         // Recenter near any edge (m)
        int submap_halfwidth = 10;                   // 子图窗口:中心关键帧 ±N
        double icp_ds = 0.4;                         // 子图体素(m)
        bool pause_on_lost = true;                   // Legacy Manager mode; false allows local recovery.
        int failure_count_before_relocalization = 3; // 连续失效帧数 → 标记 LOST
    };

    // One attempt owns its pose, quality metrics and rejection reason.
    // accepted alone controls whether the result may be published as a measurement.
    struct RegistrationResult
    {
        Eigen::Matrix4d T_map_base = Eigen::Matrix4d::Identity();
        bool accepted = false;
        bool converged = false;
        double fitness = std::numeric_limits<double>::quiet_NaN();
        double inlier_fraction = std::numeric_limits<double>::quiet_NaN();
        double correction_m = 0.0;
        double correction_yaw_deg = 0.0;
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
            const std::vector<Eigen::Matrix4d>& poses,
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

    // Own keyframe poses independently of the caller.
    std::vector<Eigen::Matrix4d> poses_;
    Config cfg_;
    KeyframeCache cache_;
    std::unique_ptr<SubmapCache> submap_cache_;
    std::unique_ptr<ScanMatcher> matcher_;

    std::mutex mtx_;
    int fail_count_ = 0;
    bool lost_ = false;
    RegistrationResult last_result_;
};

} // namespace ml_icp_loc
