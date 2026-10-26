#include "ml_relocalization/relocalizer.hpp"
#include "ml_relocalization/keyframe_map.hpp"
#include "registration.hpp"
#include "scancontext/Scancontext.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <stdexcept>

namespace ml_relocalization {
struct Relocalizer::Impl {
  Config config;
  KeyframeMap map;
  SCManager context;
  explicit Impl(const Config& value) : config(value), map(value) {
    if (!context.loadFromFile(config.map_directory + "/sc_db.bin"))
      throw std::runtime_error("failed to load Scan Context database");
    context.setSCdistThres(config.sc_dist_thres);
    if (context.getNumDescriptors() != map.size())
      throw std::runtime_error("SC descriptor / keyframe pose count mismatch");
  }
};
Relocalizer::Relocalizer(const Config& config) : impl_(std::make_unique<Impl>(config)) {}
Relocalizer::~Relocalizer() = default;

MatchResult Relocalizer::match(const Cloud::Ptr& scan_body) {
  auto& state = *impl_;
  const auto& config = state.config;
  MatchResult result;
  auto descriptor = state.context.makeScancontext(*scan_body);
  auto ring_key = state.context.makeRingkeyFromScancontext(descriptor);
  auto key = eig2stdvec(ring_key);
  SCManager::QueryDiagnostics query;
  const auto candidate = state.context.detectLoopClosureIDBetweenSession(key, descriptor, &query);
  result.candidate = query.best_candidate;
  result.sc_distance = query.distance;
  if (candidate.first < 0) { result.reason = "sc_no_candidate"; return result; }
  if (candidate.first >= static_cast<int>(state.map.size())) {
    result.reason = "candidate_out_of_range"; return result;
  }
  const double yaw = config.yaw_diff_sign * candidate.second;
  Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
  rotation(0,0) = std::cos(yaw); rotation(0,1) = -std::sin(yaw);
  rotation(1,0) = std::sin(yaw); rotation(1,1) = std::cos(yaw);
  const Eigen::Matrix4d T_map_body_hypothesis = state.map.pose(candidate.first) * rotation;
  auto source = std::make_shared<Cloud>();
  pcl::transformPointCloud(*scan_body, *source, T_map_body_hypothesis);
  if (config.debug_publish_clouds) result.hypothesis = source;
  auto raw = state.map.submap(candidate.first, config.submap_halfwidth);
  if (static_cast<int>(raw->size()) < config.submap_min_points) {
    result.reason = "submap_too_small"; return result;
  }
  auto target = std::make_shared<Cloud>();
  pcl::VoxelGrid<Point> voxel;
  voxel.setLeafSize(config.icp_ds_size, config.icp_ds_size, config.icp_ds_size);
  voxel.setInputCloud(raw);
  voxel.filter(*target);
  result.submap_points = target->size();
  if (config.debug_publish_clouds) result.submap = target;
  refine(config, source, target, T_map_body_hypothesis, result);
  return result;
}
}  // namespace ml_relocalization
