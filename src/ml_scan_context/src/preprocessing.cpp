#include "ml_scan_context/preprocessing.hpp"
#include <array>
#include <cmath>
#include <iomanip>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>

namespace ml_scan_context {
Preprocessor::Preprocessor(PreprocessingConfig config) : config_(config) {
  if (!std::isfinite(config.xy_radius) || config.xy_radius <= 0 ||
    !std::isfinite(config.height_offset) || !std::isfinite(config.voxel_size) || config.voxel_size <= 0)
    throw std::invalid_argument("invalid preprocessing configuration");
}
Points Preprocessor::apply(const Points & points) const {
  struct Accumulator { Eigen::Vector3d sum = Eigen::Vector3d::Zero(); std::size_t count = 0; };
  std::map<std::array<std::int64_t, 3>, Accumulator> voxels;
  for (const auto & raw : points) {
    if (!raw.allFinite()) continue;
    if (!(std::sqrt(raw.x()*raw.x() + raw.y()*raw.y()) < config_.xy_radius)) continue;
    Eigen::Vector3d point = raw; point.z() += config_.height_offset;
    std::array<std::int64_t, 3> key;
    for (int axis = 0; axis < 3; ++axis) {
      const double value = std::floor(point[axis] / config_.voxel_size);
      // double表示的2^63不在int64可转换范围内。
      if (!std::isfinite(value) || value < -9223372036854775808.0 || value >= 9223372036854775808.0)
        throw std::invalid_argument("point exceeds voxel key range");
      key[axis] = static_cast<std::int64_t>(value);
    }
    auto & voxel = voxels[key]; voxel.sum += point; ++voxel.count;
    if (!voxel.sum.allFinite()) throw std::overflow_error("voxel sum overflow");
  }
  Points result; result.reserve(voxels.size());
  for (const auto & entry : voxels) result.push_back(entry.second.sum / static_cast<double>(entry.second.count));
  return result;
}
std::string Preprocessor::identifier(const std::string & frame_convention) const {
  if (frame_convention.empty()) throw std::invalid_argument("empty frame convention");
  std::ostringstream out; out.imbue(std::locale::classic());
  out << std::setprecision(17) << "floor-centroid-v1;frame=" << std::quoted(frame_convention)
      << ";xy_radius=" << config_.xy_radius << ";height_offset=" << config_.height_offset
      << ";voxel_size=" << config_.voxel_size;
  return out.str();
}
}  // namespace ml_scan_context
