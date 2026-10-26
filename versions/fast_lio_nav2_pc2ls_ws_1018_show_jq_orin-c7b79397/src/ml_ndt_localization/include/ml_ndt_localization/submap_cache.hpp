#pragma once

#include "ml_ndt_localization/point_types.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <string>
#include <pcl/common/point_tests.h>

namespace ml_ndt_loc
{
// Immutable map-frame point clouds. The center describes cached map coverage,
// not a second localization estimate. No TF or accumulated pose is stored here.
class SubmapCache
{
public:
    using Cloud = pcl::PointCloud<PointType>;
    struct Selection {
        Cloud::ConstPtr cloud;
        bool updated;
        Eigen::Vector2d center;
    };

    SubmapCache(const std::string& path, double leaf, double size, double margin)
        : half_size_(size / 2.0), margin_(margin)
    {
        if (!std::isfinite(size) || size <= 0 || !std::isfinite(margin) ||
            margin <= 0 || margin >= half_size_ || !std::isfinite(leaf) || leaf <= 0)
            throw std::invalid_argument("window requires size>0, 0<margin<size/2, voxel_size>0");
        auto raw = pcl::make_shared<Cloud>();
        if (pcl::io::loadPCDFile<PointType>(path, *raw) != 0)
            throw std::runtime_error("cannot load localization map: " + path);
        auto finite = pcl::make_shared<Cloud>();
        finite->reserve(raw->size());
        for (const auto& p : *raw) if (pcl::isFinite(p)) finite->push_back(p);
        if (finite->size() < 1000) throw std::runtime_error("localization map has fewer than 1000 finite points");
        map_ = pcl::make_shared<Cloud>();
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.setInputCloud(finite);
        voxel.filter(*map_);
        if (map_->size() < 1000) throw std::runtime_error("downsampled localization map has fewer than 1000 points");
    }

    Selection Select(const Eigen::Vector2d& position)
    {
        if (!position.allFinite()) throw std::invalid_argument("nonfinite window position");
        const bool update = !window_ ||
            (position - center_).cwiseAbs().maxCoeff() >= half_size_ - margin_;
        if (update) {
            auto next = pcl::make_shared<Cloud>();
            next->reserve(map_->size());
            for (size_t i = 0; i < map_->size(); ++i) {
                const auto& p = (*map_)[i];
                if (std::abs(double(p.x) - position.x()) <= half_size_ &&
                    std::abs(double(p.y) - position.y()) <= half_size_) {
                    next->push_back(p); // Preserve z, including buildings and vegetation.
                }
            }
            next->points.shrink_to_fit();
            // Keep empty windows too: stationary out-of-map predictions must not
            // cause a full-map rescan every tick. A new position can refresh it.
            center_ = position;
            window_ = next;
        }
        return {window_, update, center_};
    }

    Cloud::ConstPtr Map() const { return map_; }

private:
    double half_size_, margin_;
    Cloud::Ptr map_;
    Cloud::ConstPtr window_;
    Eigen::Vector2d center_ = Eigen::Vector2d::Zero();
};
} // namespace ml_ndt_loc
