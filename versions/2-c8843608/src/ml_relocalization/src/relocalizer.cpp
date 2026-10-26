#include "ml_relocalization/relocalizer.hpp"
#include "ml_relocalization/keyframe_map.hpp"
#include "registration.hpp"
#include "ml_relocalization/candidate_selection.hpp"
#include "scancontext/Scancontext.h"
#include "ml_scan_context/database.hpp"
#include "ml_scan_context/preprocessing.hpp"
#include <algorithm>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <stdexcept>

namespace ml_relocalization {
struct Relocalizer::Impl {
  Config config;
  KeyframeMap map;
  std::unique_ptr<SCManager> context;
  ml_scan_context::Preprocessor preprocessing;
  std::unique_ptr<ml_scan_context::Database> polar;
  explicit Impl(const Config& value) : config(value), map(value) {
    if (config.descriptor_backend == "polar") {
      if (config.polar_threads < 1 || config.polar_database_file.empty() || config.polar_top_k < 1 || config.polar_top_k > 5 ||
          !std::isfinite(config.polar_maximum_distance) || config.polar_maximum_distance < 0.0 ||
          !std::isfinite(config.confirm_position_thresh) || config.confirm_position_thresh <= 0.0 ||
          !std::isfinite(config.confirm_yaw_thresh_deg) || config.confirm_yaw_thresh_deg <= 0.0)
        throw std::invalid_argument("invalid polar candidate configuration");
      ml_scan_context::DatabaseConfig database_config;
      database_config.preprocessing_id = preprocessing.identifier(config.polar_frame_convention);
      polar = std::make_unique<ml_scan_context::Database>(database_config);
      polar->load(config.polar_database_file);
      auto ids = polar->keyframeIds();
      std::sort(ids.begin(), ids.end());
      if (ids.size() != map.size()) throw std::runtime_error("polar / keyframe pose count mismatch");
      for (std::size_t i = 0; i < ids.size(); ++i)
        if (ids[i] != i) throw std::runtime_error("polar keyframe IDs must match pose indices");
      return;
    }
    if (config.descriptor_backend != "legacy_sc") throw std::invalid_argument("unknown descriptor backend");
    context = std::make_unique<SCManager>();
    if (!context->loadFromFile(config.map_directory + "/sc_db.bin"))
      throw std::runtime_error("failed to load Scan Context database");
    context->setSCdistThres(config.sc_dist_thres);
    if (context->getNumDescriptors() != map.size())
      throw std::runtime_error("SC descriptor / keyframe pose count mismatch");
  }
  // 两种描述子共用候选几何验证；不参与检索排序和多候选消歧。
  MatchResult matchCandidate(const Cloud::Ptr& scan_body, int candidate_id,
                             double distance, const Eigen::Matrix4d& T_map_body_hypothesis) {
    MatchResult result;
    result.candidate = candidate_id;
    result.sc_distance = distance;
    if (config.debug_publish_clouds) {
      result.hypothesis = std::make_shared<Cloud>();
      pcl::transformPointCloud(*scan_body, *result.hypothesis, T_map_body_hypothesis);
    }
    auto raw = map.submap(candidate_id, config.submap_halfwidth);
    if (static_cast<int>(raw->size()) < config.submap_min_points) {
      result.reason = "submap_too_small"; return result;
    }
    auto target = std::make_shared<Cloud>();
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(config.registration_voxel_size, config.registration_voxel_size, config.registration_voxel_size);
    voxel.setInputCloud(raw);
    voxel.filter(*target);
    result.submap_points = target->size();
    if (config.debug_publish_clouds) result.submap = target;
    refine(config, scan_body, target, T_map_body_hypothesis, result);
    result.candidates_tested = 1;
    result.candidates_accepted = result.valid ? 1 : 0;
    return result;
  }

};
Relocalizer::Relocalizer(const Config& config) : impl_(std::make_unique<Impl>(config)) {}
Relocalizer::~Relocalizer() = default;

MatchResult Relocalizer::match(const Cloud::Ptr& scan_body, const Cloud::Ptr& descriptor_scan) {
  auto& state = *impl_;
  const auto& config = state.config;
  MatchResult result;
  if (!scan_body || scan_body->empty()) { result.reason = "registration_input_empty"; return result; }
  if (state.polar) {
    if (!descriptor_scan || descriptor_scan->empty()) {
      result.reason = "polar_raw_input_missing"; return result;
    }
    ml_scan_context::Points points;
    points.reserve(descriptor_scan->size());
    for (const auto& p : *descriptor_scan) points.emplace_back(p.x, p.y, p.z);
    ml_scan_context::QueryOptions options;
    options.threads = config.polar_threads;
    options.top_k = config.polar_top_k;
    options.maximum_distance = config.polar_maximum_distance;
    const auto candidates = state.polar->query(state.preprocessing.apply(points), options);
    if (candidates.empty()) { result.reason = "polar_no_candidate"; return result; }
    std::vector<MatchResult> accepted;
    for (const auto& candidate : candidates) {
      // 新接口直接提供keyframe<-query，不应用旧SC的yaw_diff_sign。
      auto trial = state.matchCandidate(scan_body, static_cast<int>(candidate.keyframe_id), candidate.descriptor_distance,
        state.map.pose(static_cast<int>(candidate.keyframe_id)) * candidate.T_keyframe_query.matrix());
      if (trial.valid) accepted.push_back(trial);
      if (result.candidate < 0) result = trial;
    }
    if (!accepted.empty()) result = SelectPolarCandidate(accepted, config);
    result.candidates_tested = candidates.size();
    result.candidates_accepted = accepted.size();
    return result;
  }
  auto descriptor = state.context->makeScancontext(*scan_body);
  auto ring_key = state.context->makeRingkeyFromScancontext(descriptor);
  auto key = eig2stdvec(ring_key);
  SCManager::QueryDiagnostics query;
  const auto candidate = state.context->detectLoopClosureIDBetweenSession(key, descriptor, &query);
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
  return state.matchCandidate(scan_body, candidate.first, query.distance, T_map_body_hypothesis);
}
}  // namespace ml_relocalization
