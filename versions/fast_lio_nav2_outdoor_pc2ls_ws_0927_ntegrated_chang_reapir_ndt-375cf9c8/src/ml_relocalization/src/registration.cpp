#include "registration.hpp"
#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>
#include <pcl/common/point_tests.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <stdexcept>

namespace ml_relocalization {
void ValidateRegistrationConfig(const Config& c) {
  for (double v : {c.ndt_resolution, c.ndt_step_size, c.ndt_transformation_epsilon,
                   c.max_translation_correction, c.max_yaw_correction_deg})
    if (!std::isfinite(v) || v <= 0.) throw std::invalid_argument("invalid NDT scale or correction limit");
  if (c.ndt_threads < 1 || c.ndt_max_iterations < 2 ||
      !std::isfinite(c.ndt_likelihood_threshold) || c.ndt_likelihood_threshold < 0. ||
      !std::isfinite(c.inlier_fraction_threshold) || c.inlier_fraction_threshold < 0. ||
      c.inlier_fraction_threshold > 1.) throw std::invalid_argument("invalid NDT quality parameters");
}
void refine(const Config& config, const Cloud::Ptr& source, const Cloud::Ptr& target,
            const Eigen::Matrix4d& initial, MatchResult& result) {
  ValidateRegistrationConfig(config);
  if (!source || source->empty() || !target || target->empty()) {
    result.reason = "registration_input_empty"; return;
  }
  if (!initial.allFinite()) { result.reason = "initial_pose_nonfinite"; return; }
  pclomp::MultiGridNormalDistributionsTransform<Point, Point> ndt;
  pclomp::NdtParams params;
  params.num_threads = config.ndt_threads;
  params.resolution = config.ndt_resolution;
  params.step_size = config.ndt_step_size;
  params.trans_epsilon = config.ndt_transformation_epsilon;
  params.max_iterations = config.ndt_max_iterations;
  params.regularization_scale_factor = 0.;
  params.search_method = pclomp::KDTREE;
  params.use_line_search = false;
  ndt.setParams(params);
  ndt.setInputTarget(target);
  auto aligned = std::make_shared<Cloud>();
  ndt.align(*aligned, initial.cast<float>(), source);
  result.registration_ran = true;
  if (config.debug_publish_clouds) result.aligned = aligned;
  // NDT 直接返回 T_map_body，不能再与 SC 初值相乘。
  result.T_map_body = ndt.getFinalTransformation().cast<double>();
  result.iterations = ndt.getFinalNumIteration();
  result.likelihood = ndt.getNearestVoxelTransformationLikelihood();
  result.transformation_probability = ndt.getTransformationProbability();
  result.converged = result.iterations > 0 && result.iterations < config.ndt_max_iterations;
  result.correction_m = (result.T_map_body.block<3,1>(0,3)-initial.block<3,1>(0,3)).norm();
  // 保留原有 map 系相对旋转语义，避免本轮顺便改变航向修正门控。
  const Eigen::Matrix4d delta = result.T_map_body * initial.inverse();
  result.correction_yaw_deg = std::abs(std::atan2(delta(1,0),delta(0,0))*180./M_PI);
  pcl::KdTreeFLANN<Point> tree;
  tree.setInputCloud(target);
  std::vector<int> indices(1); std::vector<float> distances(1);
  size_t count = 0;
  for (const auto& p : *aligned)
    if (pcl::isFinite(p) && tree.nearestKSearch(p,1,indices,distances)>0 && distances[0]<1.f) ++count;
  result.inlier_fraction = double(count)/source->size();
  if (!result.T_map_body.allFinite() || !std::isfinite(result.likelihood) ||
      !std::isfinite(result.transformation_probability) || !std::isfinite(result.correction_m) ||
      !std::isfinite(result.correction_yaw_deg)) result.reason = "registration_nonfinite";
  else if (!result.converged) result.reason = "ndt_not_converged";
  else if (result.likelihood < config.ndt_likelihood_threshold) result.reason = "ndt_likelihood";
  else if (result.inlier_fraction < config.inlier_fraction_threshold) result.reason = "map_coverage";
  else if (result.correction_m > config.max_translation_correction) result.reason = "translation_correction";
  else if (result.correction_yaw_deg > config.max_yaw_correction_deg) result.reason = "yaw_correction";
  result.valid = result.reason.empty();
}
}  // namespace ml_relocalization
