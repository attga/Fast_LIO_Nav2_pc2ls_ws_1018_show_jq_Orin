#include "utility.hpp"
#include "Scancontext.h"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>
#include <chrono>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace fs = std::filesystem;

class ScPose : public ParamServer
{
private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_initial_odom;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_initial_pose_stamped;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_custom_cloud;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud;

    std::string map_data_dir;
    int num_keyframes = 0;
    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<std::vector<float>> polarcontext_invkeys_mat_;
    std::unique_ptr<KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<float>>, float>> polarcontext_tree_;
    Eigen::MatrixXd keyframe_poses;

    int PC_NUM_RING = 20;
    int PC_NUM_SECTOR = 60;
    double LIDAR_HEIGHT = 2.0;
    double PC_MAX_RADIUS = 80.0;
    int NUM_CANDIDATES_FROM_TREE = 5;
    double SEARCH_RATIO = 0.1;
    double SC_DIST_THRES = 0.3;
    double PC_UNIT_SECTORANGLE = 6.0;

    bool auto_reloc_once = false;
    std::atomic<bool> reloc_success{false};

    // Asynchronous decoupled queue to avoid blocking ROS spin thread
    std::mutex cloud_buf_mutex;
    std::condition_variable cloud_cv;
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_scan;
    std_msgs::msg::Header latest_header;
    bool has_new_scan = false;

public:
    ScPose(const rclcpp::NodeOptions & options) : ParamServer("lio_sam_sc_pose", options)
    {
        declare_parameter("scDistThres", 0.3);
        get_parameter("scDistThres", SC_DIST_THRES);

        declare_parameter("scNumCandidates", 5);
        get_parameter("scNumCandidates", NUM_CANDIDATES_FROM_TREE);

        declare_parameter("scNumRing", 20);
        get_parameter("scNumRing", PC_NUM_RING);

        declare_parameter("scNumSector", 60);
        get_parameter("scNumSector", PC_NUM_SECTOR);

        declare_parameter("scMaxRadius", 80.0);
        get_parameter("scMaxRadius", PC_MAX_RADIUS);

        declare_parameter("scLidarHeight", 2.0);
        get_parameter("scLidarHeight", LIDAR_HEIGHT);

        declare_parameter("autoRelocOnce", false);
        get_parameter("autoRelocOnce", auto_reloc_once);

        PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);

        // Determine map data directory
        declare_parameter("mapDataDir", "");
        get_parameter("mapDataDir", map_data_dir);
        if (map_data_dir.empty())
        {
            char abs_path[PATH_MAX];
            if (realpath(__FILE__, abs_path) != nullptr)
            {
                std::string path_str(abs_path);
                path_str = path_str.substr(0, path_str.find_last_of("/"));
                path_str = path_str.substr(0, path_str.find_last_of("/"));
                map_data_dir = path_str + "/data/";
            }
            else
            {
                map_data_dir = savePCDDirectory;
            }
        }
        if (!map_data_dir.empty() && map_data_dir.back() != '/')
            map_data_dir += "/";

        RCLCPP_INFO(get_logger(), "ScanContext map data directory: %s", map_data_dir.c_str());

        auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        pub_initial_odom = create_publisher<nav_msgs::msg::Odometry>("initial_odom", qos_reliable);
        pub_initial_pose_stamped = create_publisher<geometry_msgs::msg::PoseStamped>("initial_pose", qos_reliable);

        // Subscribe to Mid-360 custom msg and PointCloud2
        sub_custom_cloud = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            pointCloudTopic, qos_lidar,
            std::bind(&ScPose::customCloudCallback, this, std::placeholders::_1));

        sub_point_cloud = create_subscription<sensor_msgs::msg::PointCloud2>(
            "points_raw", qos_lidar,
            std::bind(&ScPose::pointCloud2Callback, this, std::placeholders::_1));

        loadScanContextDatabase();
    }

    void stop()
    {
        cloud_cv.notify_all();
    }

    void loadScanContextDatabase()
    {
        std::string scd_dir = map_data_dir + "SCDs/";
        std::string pose_file = map_data_dir + "pose.txt";

        if (!fs::exists(pose_file))
        {
            RCLCPP_ERROR(get_logger(), "Pose file does not exist: %s", pose_file.c_str());
            return;
        }

        std::ifstream pf(pose_file);
        std::string line;
        std::vector<std::vector<double>> pose_vec;
        while (std::getline(pf, line))
        {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::vector<double> row(6);
            for (int i = 0; i < 6; ++i)
                ss >> row[i];
            pose_vec.push_back(row);
        }
        pf.close();

        num_keyframes = pose_vec.size();
        if (num_keyframes == 0)
        {
            RCLCPP_ERROR(get_logger(), "No keyframes loaded from %s", pose_file.c_str());
            return;
        }

        keyframe_poses.resize(num_keyframes, 6);
        for (int i = 0; i < num_keyframes; ++i)
            for (int j = 0; j < 6; ++j)
                keyframe_poses(i, j) = pose_vec[i][j];

        RCLCPP_INFO(get_logger(), "Loaded %d keyframe poses from pose.txt", num_keyframes);

        auto scd2matrix = [](const std::string& path, int a, int b) -> Eigen::MatrixXd {
            std::ifstream data(path);
            Eigen::MatrixXd m_matrix(a, b);
            Eigen::VectorXd hang(b);
            for (int j = 0; j < a; ++j) {
                for (int i = 0; i < b; ++i) {
                    data >> hang(i);
                }
                m_matrix.row(j) = hang;
            }
            return m_matrix;
        };

        polarcontexts_.clear();
        polarcontext_invkeys_.clear();
        polarcontext_invkeys_mat_.clear();

        int loaded_scd_count = 0;
        for (int i = 0; i < num_keyframes; ++i)
        {
            std::string scd_name = padZeros(i, 6) + ".scd";
            std::string scd_path = scd_dir + scd_name;

            if (!fs::exists(scd_path))
            {
                RCLCPP_WARN(get_logger(), "SCD file %s does not exist, stopping SCD load at %d", scd_path.c_str(), i);
                break;
            }

            Eigen::MatrixXd sc = scd2matrix(scd_path, PC_NUM_RING, PC_NUM_SECTOR);
            Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
            std::vector<float> polarcontext_invkey_vec(ringkey.data(), ringkey.data() + ringkey.size());

            polarcontexts_.push_back(sc);
            polarcontext_invkeys_.push_back(ringkey);
            polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec);
            loaded_scd_count++;
        }

        if (loaded_scd_count == 0)
        {
            RCLCPP_ERROR(get_logger(), "Failed to load any SCD files from %s", scd_dir.c_str());
            return;
        }

        polarcontext_tree_ = std::make_unique<KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<float>>, float>>(
            PC_NUM_RING, polarcontext_invkeys_mat_, 10);

        RCLCPP_INFO(get_logger(), "\033[1;32m----> ScanContext Database loaded successfully (%d keyframes). KD-Tree built.\033[0m", loaded_scd_count);
    }

    void customCloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        if (auto_reloc_once && reloc_success.load())
            return;

        RCLCPP_INFO_ONCE(get_logger(), "First CustomMsg point cloud received in sc_pose. Searching for initial pose...");

        pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>());
        scan->reserve(msg->point_num);
        pcl::PointXYZI pt;
        const float min_r2 = lidarMinRange * lidarMinRange;
        const float max_r2 = lidarMaxRange * lidarMaxRange;
        for (uint i = 0; i < msg->point_num; ++i)
        {
            pt.x = msg->points[i].x;
            pt.y = msg->points[i].y;
            pt.z = msg->points[i].z;
            pt.intensity = msg->points[i].reflectivity;
            float r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            if (r2 >= min_r2 && r2 <= max_r2)
                scan->push_back(pt);
        }

        {
            std::lock_guard<std::mutex> lock(cloud_buf_mutex);
            latest_scan = scan;
            latest_header = msg->header;
            has_new_scan = true;
        }
        cloud_cv.notify_one();
    }

    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (auto_reloc_once && reloc_success.load())
            return;

        RCLCPP_INFO_ONCE(get_logger(), "First PointCloud2 received in sc_pose. Searching for initial pose...");

        pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *scan);

        {
            std::lock_guard<std::mutex> lock(cloud_buf_mutex);
            latest_scan = scan;
            latest_header = msg->header;
            has_new_scan = true;
        }
        cloud_cv.notify_one();
    }

    void processThread()
    {
        while (rclcpp::ok())
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr scan;
            std_msgs::msg::Header header;
            {
                std::unique_lock<std::mutex> lock(cloud_buf_mutex);
                cloud_cv.wait(lock, [this]() {
                    return !rclcpp::ok() || (has_new_scan && !(auto_reloc_once && reloc_success.load()));
                });
                if (!rclcpp::ok()) break;

                scan = latest_scan;
                header = latest_header;
                has_new_scan = false;
            }

            if (auto_reloc_once && reloc_success.load())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }

            if (scan && !scan->empty())
            {
                processScan(*scan, header);
            }
        }
    }

    void processScan(pcl::PointCloud<pcl::PointXYZI>& scan, const std_msgs::msg::Header& header)
    {
        if (scan.empty() || !polarcontext_tree_ || polarcontexts_.empty())
            return;

        auto start_time = std::chrono::steady_clock::now();

        Eigen::MatrixXd curr_desc = makeScancontext(scan);
        Eigen::MatrixXd curr_ringkey = makeRingkeyFromScancontext(curr_desc);
        std::vector<float> curr_key(curr_ringkey.data(), curr_ringkey.data() + curr_ringkey.size());

        int num_candidates = std::min(NUM_CANDIDATES_FROM_TREE, (int)polarcontexts_.size());
        std::vector<size_t> candidate_indexes(num_candidates);
        std::vector<float> out_dists_sqr(num_candidates);

        nanoflann::KNNResultSet<float> knnsearch_result(num_candidates);
        knnsearch_result.init(&candidate_indexes[0], &out_dists_sqr[0]);
        polarcontext_tree_->index->findNeighbors(knnsearch_result, &curr_key[0], nanoflann::SearchParams(10));

        double min_dist = 1e7;
        int nn_align = 0;
        int nn_idx = 0;

        for (int i = 0; i < num_candidates; ++i)
        {
            Eigen::MatrixXd polarcontext_candidate = polarcontexts_[candidate_indexes[i]];
            std::pair<double, int> sc_dist_result = distanceBtnScanContext(curr_desc, polarcontext_candidate);
            double candidate_dist = sc_dist_result.first;
            int candidate_align = sc_dist_result.second;
            if (candidate_dist < min_dist)
            {
                min_dist = candidate_dist;
                nn_align = candidate_align;
                nn_idx = candidate_indexes[i];
            }
        }

        auto end_time = std::chrono::steady_clock::now();
        double cost_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        static int last_matched_keyframe = -1;
        static int confirm_hits = 0;

        if (min_dist < SC_DIST_THRES)
        {
            if (last_matched_keyframe == -1 || std::abs(nn_idx - last_matched_keyframe) <= 3)
            {
                confirm_hits++;
            }
            else
            {
                confirm_hits = 1;
            }
            last_matched_keyframe = nn_idx;

            // Require 2 consecutive matching frames or very high confidence (dist < 0.14)
            if (confirm_hits >= 2 || min_dist < 0.14)
            {
                float yaw_diff_rad = nn_align * PC_UNIT_SECTORANGLE * M_PI / 180.0;
                double roll = keyframe_poses(nn_idx, 3);
                double pitch = keyframe_poses(nn_idx, 4);
                double yaw = keyframe_poses(nn_idx, 5) - yaw_diff_rad;

                // Normalize yaw to [-pi, pi]
                while (yaw > M_PI) yaw -= 2.0 * M_PI;
                while (yaw < -M_PI) yaw += 2.0 * M_PI;

                tf2::Quaternion q;
                q.setRPY(roll, pitch, yaw);

                nav_msgs::msg::Odometry msgOdom;
                msgOdom.header.frame_id = mapFrame;
                msgOdom.header.stamp = header.stamp;
                msgOdom.child_frame_id = baselinkFrame;
                msgOdom.pose.pose.position.x = keyframe_poses(nn_idx, 0);
                msgOdom.pose.pose.position.y = keyframe_poses(nn_idx, 1);
                msgOdom.pose.pose.position.z = keyframe_poses(nn_idx, 2);
                msgOdom.pose.pose.orientation.x = q.x();
                msgOdom.pose.pose.orientation.y = q.y();
                msgOdom.pose.pose.orientation.z = q.z();
                msgOdom.pose.pose.orientation.w = q.w();

                msgOdom.pose.covariance[0] = min_dist;

                pub_initial_odom->publish(msgOdom);

                geometry_msgs::msg::PoseStamped msgPose;
                msgPose.header = msgOdom.header;
                msgPose.pose = msgOdom.pose.pose;
                pub_initial_pose_stamped->publish(msgPose);

                reloc_success.store(true);

                RCLCPP_INFO(get_logger(),
                    "\033[1;33m[Initial Pose]\033[0m X: %7.2f m, Y: %7.2f m, Z: %7.2f m | Roll: %6.1f°, Pitch: %6.1f°, Yaw: %6.1f° (SC Dist: %.3f, Search: %.1fms)",
                    keyframe_poses(nn_idx, 0), keyframe_poses(nn_idx, 1), keyframe_poses(nn_idx, 2),
                    roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI, min_dist, cost_ms);
            }
        }
        else
        {
            confirm_hits = 0;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[SC Searching] Nearest Keyframe: #%d, Best Dist: %.4f (thres: %.2f) | Query: %.1fms",
                nn_idx, min_dist, SC_DIST_THRES, cost_ms);
        }
    }

    Eigen::MatrixXd makeScancontext(const pcl::PointCloud<pcl::PointXYZI>& _scan_down)
    {
        int num_pts = _scan_down.points.size();
        const int NO_POINT = -1000;
        Eigen::MatrixXd desc = NO_POINT * Eigen::MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

        for (int pt_idx = 0; pt_idx < num_pts; ++pt_idx)
        {
            float pt_x = _scan_down.points[pt_idx].x;
            float pt_y = _scan_down.points[pt_idx].y;
            float pt_z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT;
            float azim_range = sqrt(pt_x * pt_x + pt_y * pt_y);
            if (azim_range > PC_MAX_RADIUS || azim_range < 0.1) continue;

            float azim_angle = atan2(pt_y, pt_x) * 180.0 / M_PI;
            if (azim_angle < 0) azim_angle += 360.0;

            int ring_idx = std::max(std::min(PC_NUM_RING, (int)ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING)), 1);
            int sctor_idx = std::max(std::min(PC_NUM_SECTOR, (int)ceil((azim_angle / 360.0) * PC_NUM_SECTOR)), 1);

            if (desc(ring_idx - 1, sctor_idx - 1) < pt_z)
                desc(ring_idx - 1, sctor_idx - 1) = pt_z;
        }

        for (int row_idx = 0; row_idx < desc.rows(); ++row_idx)
            for (int col_idx = 0; col_idx < desc.cols(); ++col_idx)
                if (desc(row_idx, col_idx) == NO_POINT)
                    desc(row_idx, col_idx) = 0;

        return desc;
    }

    Eigen::MatrixXd makeRingkeyFromScancontext(const Eigen::MatrixXd& _desc)
    {
        Eigen::MatrixXd invariant_key(_desc.rows(), 1);
        for (int row_idx = 0; row_idx < _desc.rows(); ++row_idx)
        {
            Eigen::MatrixXd curr_row = _desc.row(row_idx);
            invariant_key(row_idx, 0) = curr_row.mean();
        }
        return invariant_key;
    }

    Eigen::MatrixXd makeSectorkeyFromScancontext(const Eigen::MatrixXd& _desc)
    {
        Eigen::MatrixXd variant_key(1, _desc.cols());
        for (int col_idx = 0; col_idx < _desc.cols(); ++col_idx)
        {
            Eigen::MatrixXd curr_col = _desc.col(col_idx);
            variant_key(0, col_idx) = curr_col.mean();
        }
        return variant_key;
    }

    std::pair<double, int> distanceBtnScanContext(Eigen::MatrixXd& _sc1, Eigen::MatrixXd& _sc2)
    {
        Eigen::MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(_sc1);
        Eigen::MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(_sc2);
        int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);

        const int SEARCH_RADIUS = std::round(0.5 * SEARCH_RATIO * _sc1.cols());
        std::vector<int> shift_idx_search_space{argmin_vkey_shift};
        for (int ii = 1; ii <= SEARCH_RADIUS; ++ii)
        {
            shift_idx_search_space.push_back((argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols());
            shift_idx_search_space.push_back((argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols());
        }
        std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

        int argmin_shift = 0;
        double min_sc_dist = 1e7;
        for (int num_shift : shift_idx_search_space)
        {
            Eigen::MatrixXd sc2_shifted = circshift(_sc2, num_shift);
            double cur_sc_dist = distDirectSC(_sc1, sc2_shifted);
            if (cur_sc_dist < min_sc_dist)
            {
                argmin_shift = num_shift;
                min_sc_dist = cur_sc_dist;
            }
        }
        return std::make_pair(min_sc_dist, argmin_shift);
    }

    int fastAlignUsingVkey(Eigen::MatrixXd& _vkey1, Eigen::MatrixXd& _vkey2)
    {
        int argmin_vkey_shift = 0;
        double min_vkey_diff_norm = 1e7;
        for (int shift_idx = 0; shift_idx < _vkey1.cols(); ++shift_idx)
        {
            Eigen::MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);
            Eigen::MatrixXd vkey_diff = _vkey1 - vkey2_shifted;
            double cur_diff_norm = vkey_diff.norm();
            if (cur_diff_norm < min_vkey_diff_norm)
            {
                argmin_vkey_shift = shift_idx;
                min_vkey_diff_norm = cur_diff_norm;
            }
        }
        return argmin_vkey_shift;
    }

    Eigen::MatrixXd circshift(const Eigen::MatrixXd& _mat, int _num_shift)
    {
        assert(_num_shift >= 0);
        if (_num_shift == 0)
            return _mat;

        Eigen::MatrixXd shifted_mat = Eigen::MatrixXd::Zero(_mat.rows(), _mat.cols());
        for (int col_idx = 0; col_idx < _mat.cols(); ++col_idx)
        {
            int new_location = (col_idx + _num_shift) % _mat.cols();
            shifted_mat.col(new_location) = _mat.col(col_idx);
        }
        return shifted_mat;
    }

    double distDirectSC(const Eigen::MatrixXd& _sc1, const Eigen::MatrixXd& _sc2)
    {
        int num_eff_cols = 0;
        double sum_sector_similarity = 0;
        for (int col_idx = 0; col_idx < _sc1.cols(); ++col_idx)
        {
            Eigen::VectorXd col_sc1 = _sc1.col(col_idx);
            Eigen::VectorXd col_sc2 = _sc2.col(col_idx);
            if (col_sc1.norm() == 0 || col_sc2.norm() == 0)
                continue;
            double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());
            sum_sector_similarity += sector_similarity;
            num_eff_cols++;
        }
        if (num_eff_cols == 0) return 1.0;
        double sc_sim = sum_sector_similarity / num_eff_cols;
        return 1.0 - sc_sim;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(false);

    auto node = std::make_shared<ScPose>(options);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> ScanContext Auto-Relocalization (sc_pose) Node Started.\033[0m");

    std::thread process_thread(&ScPose::processThread, node);

    rclcpp::spin(node);

    rclcpp::shutdown();
    node->stop();

    if (process_thread.joinable())
        process_thread.join();

    return 0;
}
