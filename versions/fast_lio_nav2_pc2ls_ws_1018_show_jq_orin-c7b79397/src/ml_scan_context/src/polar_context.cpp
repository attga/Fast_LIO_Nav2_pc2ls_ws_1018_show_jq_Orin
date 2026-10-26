#include "ml_scan_context/polar_context.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ml_scan_context {
PolarContext::PolarContext(PolarConfig config) : config_(config) {
  if (config.rings <= 0 || config.rings > 1024 || config.sectors <= 0 || config.sectors > 4096 ||
    !std::isfinite(config.radius) || config.radius <= 0 || config.minimum_points <= 0) {
    throw std::invalid_argument("invalid polar configuration");
  }
}
Descriptor PolarContext::describe(const Points & points, const Eigen::Vector2d & viewpoint) const {
  if (!viewpoint.allFinite()) throw std::invalid_argument("nonfinite viewpoint");
  Descriptor result = Descriptor::Constant(config_.rings, config_.sectors, -10000.0);
  Eigen::MatrixXi counts = Eigen::MatrixXi::Zero(config_.rings, config_.sectors);
  const double pi = std::acos(-1.0);
  for (const auto & point : points) {
    if (!point.allFinite()) continue;
    const auto xy = (point.head<2>() - viewpoint).eval();
    if (!xy.allFinite()) continue;
    const double radial = std::hypot(xy.x(), xy.y()) / (config_.radius / config_.rings);
    const int ring = radial >= config_.rings ? config_.rings - 1 : static_cast<int>(std::floor(radial));
    double angle = std::atan2(xy.y(), xy.x()) * 180.0 / pi;
    if (angle < 0) angle += 360.0;
    const int sector = std::clamp(static_cast<int>(std::ceil(angle / (360.0 / config_.sectors))) - 1, 0, config_.sectors - 1);
    ++counts(ring, sector);
    result(ring, sector) = std::max(result(ring, sector), point.z());
  }
  for (int r = 0; r < config_.rings; ++r)
    for (int c = 0; c < config_.sectors; ++c)
      if (counts(r, c) < config_.minimum_points) result(r, c) = 0;
  return result;
}
PreparedDescriptor PolarContext::prepare(const Descriptor & descriptor) const {
  if (descriptor.rows() != config_.rings || descriptor.cols() != config_.sectors || !descriptor.allFinite())
    throw std::invalid_argument("invalid descriptor");
  PreparedDescriptor result;
  result.normalized_ = descriptor;
  result.occupied_.resize(config_.sectors);
  for (int c = 0; c < config_.sectors; ++c) {
    const double norm = result.normalized_.col(c).stableNorm();
    if (norm > 0) result.normalized_.col(c) /= norm;
    result.occupied_[c] = !result.normalized_.col(c).isZero(0);
  }
  return result;
}
Alignment PolarContext::compare(const Descriptor & query, const Descriptor & target) const {
  return comparePrepared(prepare(query), prepare(target));
}
Alignment PolarContext::comparePrepared(const PreparedDescriptor & query, const PreparedDescriptor & target) const {
  const auto & q = query.normalized_;
  const auto & t = target.normalized_;
  if (q.rows() != config_.rings || t.rows() != config_.rings ||
      q.cols() != config_.sectors || t.cols() != config_.sectors)
    throw std::invalid_argument("incompatible prepared descriptor");
  Alignment best;
  for (int shift = 1; shift <= config_.sectors; ++shift) {
    double sum = 0; int engaged = 0;
    for (int c = 0; c < config_.sectors; ++c) {
      const int qc = (c - shift + config_.sectors) % config_.sectors;
      if (!query.occupied_[qc] || !target.occupied_[c]) continue;
      sum += q.col(qc).dot(t.col(c)); ++engaged;
    }
    if (engaged) {
      const double distance = 1.0 - sum / engaged;
      if (distance < best.distance) best = {distance, shift % config_.sectors};
    }
  }
  return best;
}
}  // namespace ml_scan_context
