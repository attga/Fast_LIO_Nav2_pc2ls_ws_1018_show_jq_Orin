#include "ml_scan_context/database.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <optional>
#include <exception>

namespace ml_scan_context {
Database::Database(DatabaseConfig config) : config_(std::move(config)), context_(config_.polar) {
  if (config_.preprocessing_id.empty() || config_.viewpoints.empty() || config_.viewpoints.size() > 1024)
    throw std::invalid_argument("database requires preprocessing id and viewpoints");
  for (const auto & v : config_.viewpoints)
    if (!v.allFinite()) throw std::invalid_argument("nonfinite viewpoint");
}
void Database::add(KeyframeId id, const Points & points) {
  if (std::any_of(frames_.begin(), frames_.end(), [id](const Frame & f) { return f.id == id; }))
    throw std::invalid_argument("duplicate keyframe id");
  Frame frame{id, {}, {}};
  for (const auto & v : config_.viewpoints) {
    frame.descriptors.push_back(context_.describe(points, v));
    frame.prepared.push_back(context_.prepare(frame.descriptors.back()));
  }
  frames_.push_back(std::move(frame));
}
std::vector<Candidate> Database::query(const Points & points, const QueryOptions & options) const {
  if (!std::isfinite(options.maximum_distance) || options.maximum_distance < 0)
    throw std::invalid_argument("invalid query distance");
  if (options.threads < 1) throw std::invalid_argument("query threads must be positive");
  if (options.top_k == 0) return {};
  const auto query = context_.prepare(context_.describe(points));
  const std::unordered_set<KeyframeId> allowed(options.allowed_keyframes.begin(), options.allowed_keyframes.end());
  std::vector<Candidate> candidates;
  std::vector<std::optional<Candidate>> slots(frames_.size());
  std::vector<std::exception_ptr> errors(frames_.size());
#pragma omp parallel for schedule(static) num_threads(options.threads) if(options.threads > 1)
  for (std::ptrdiff_t index = 0; index < static_cast<std::ptrdiff_t>(frames_.size()); ++index) {
    try {
      const auto & frame = frames_[index];
      if (!allowed.empty() && !allowed.count(frame.id)) continue;
      Alignment best; std::size_t view = 0;
      for (std::size_t i = 0; i < frame.descriptors.size(); ++i) {
        const auto match = context_.comparePrepared(query, frame.prepared[i]);
        if (match.distance < best.distance) { best = match; view = i; }
      }
      if (!(best.distance < options.maximum_distance)) continue;
      Candidate candidate{frame.id, best.distance, Eigen::Isometry3d::Identity()};
      candidate.T_keyframe_query.translation().head<2>() = config_.viewpoints[view];
      candidate.T_keyframe_query.linear() = Eigen::AngleAxisd(
        best.column_shift * 2.0 * std::acos(-1.0) / config_.polar.sectors,
        Eigen::Vector3d::UnitZ()).toRotationMatrix();
      slots[index] = candidate;
    } catch (...) { errors[index] = std::current_exception(); }
  }
  // 固定插入顺序归并，不让线程完成顺序影响同分排名。
  for (std::size_t i = 0; i < slots.size(); ++i) {
    if (errors[i]) std::rethrow_exception(errors[i]);
    if (slots[i]) candidates.push_back(*slots[i]);
  }
  // 同分保留插入顺序，匹配冻结实验的稳定排序。
  std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
    return a.descriptor_distance < b.descriptor_distance;
  });
  if (candidates.size() > options.top_k) candidates.resize(options.top_k);
  return candidates;
}
std::vector<KeyframeId> Database::keyframeIds() const {
  std::vector<KeyframeId> ids;
  ids.reserve(frames_.size());
  for (const auto & frame : frames_) ids.push_back(frame.id);
  return ids;
}
}
