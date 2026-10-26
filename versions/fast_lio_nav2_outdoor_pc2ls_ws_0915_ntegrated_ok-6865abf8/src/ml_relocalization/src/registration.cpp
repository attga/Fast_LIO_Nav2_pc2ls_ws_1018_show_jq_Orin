#include "registration.hpp"
#include <pcl/common/eigen.h>
#include <pcl/registration/icp.h>
#include <cmath>

namespace ml_relocalization {
void refine(const Config& config, const Cloud::Ptr& source, const Cloud::Ptr& target,
            const Eigen::Matrix4d& T_map_body_hypothesis, MatchResult& result) {
  pcl::IterativeClosestPoint<Point, Point> icp;
  icp.setMaxCorrespondenceDistance(config.icp_max_corr_dist);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);
  icp.setInputSource(source);
  icp.setInputTarget(target);
  auto aligned = std::make_shared<Cloud>();
  icp.align(*aligned);  // Identity initialization; source is already placed in map.
  if (config.debug_publish_clouds) result.aligned = aligned;
  const Eigen::Matrix4f correction = icp.getFinalTransformation();
  float dx, dy, dz, roll, pitch, yaw;
  pcl::getTranslationAndEulerAngles(Eigen::Affine3f(correction), dx, dy, dz, roll, pitch, yaw);
  // source 已由 T_map_body_hypothesis 放入 map 系；ICP 求 source→target 修正。
  // T_map_body = T_icp_correction * T_map_body_hypothesis，修正必须左乘。
  result.T_map_body = correction.cast<double>() * T_map_body_hypothesis;
  result.correction_m = (result.T_map_body.block<3,1>(0,3) -
                        T_map_body_hypothesis.block<3,1>(0,3)).norm();
  const double yaw_deg = yaw * 180.0 / M_PI;
  result.correction_yaw_deg = std::fabs(yaw_deg);
  // PCL compares squared nearest-neighbor distances to this limit.
  // Truncate correspondence distance to reject far background points outside submap.
  const double max_corr_sq = config.icp_max_corr_dist * config.icp_max_corr_dist;
  const float fitness = static_cast<float>(icp.getFitnessScore(max_corr_sq));
  result.fitness = fitness;
  result.icp_ran = true;
  result.converged = icp.hasConverged();
  if (!result.converged) result.reason = "icp_not_converged";
  else if (!std::isfinite(fitness)) result.reason = "fitness_nonfinite";
  else if (fitness > config.icp_fitness_thres)
    result.reason = "fitness score=" + std::to_string(fitness);
  else if (!std::isfinite(result.correction_m) || !std::isfinite(yaw))
    result.reason = "correction_nonfinite";
  else if (result.correction_m > config.icp_t_max)
    result.reason = "translation_correction value=" + std::to_string(result.correction_m);
  else if (std::fabs(yaw_deg) > config.icp_yaw_max_deg)
    result.reason = "yaw_correction_deg value=" + std::to_string(yaw_deg);
  else if (!correction.allFinite()) result.reason = "icp_matrix_nonfinite";
  else if (!result.T_map_body.allFinite()) result.reason = "final_pose_nonfinite";
  result.valid = result.reason.empty();
}
}  // namespace ml_relocalization
