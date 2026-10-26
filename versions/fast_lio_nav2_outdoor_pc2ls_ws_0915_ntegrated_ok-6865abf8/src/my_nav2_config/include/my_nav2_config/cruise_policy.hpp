#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace my_nav2_config
{
struct Point {double x, y;};
struct CruiseState
{
  double x, y, yaw, vx, wz, scan_age;
};
struct GridView
{
  const unsigned char * cells;
  std::size_t width, height;
  double resolution, origin_x, origin_y;
};
struct CruiseSettings
{
  double target_speed{0.8};
  double cost_weight{8.0};
  double clearance_margin{0.15};
  double scan_timeout{0.3};
};
inline bool valid(const CruiseSettings & s)
{
  return std::isfinite(s.target_speed) && s.target_speed > 0.0 &&
         std::isfinite(s.cost_weight) && s.cost_weight >= 0.0 && s.cost_weight <= 100.0 &&
         std::isfinite(s.clearance_margin) && s.clearance_margin >= 0.0 &&
         std::isfinite(s.scan_timeout) && s.scan_timeout > 0.0;
}

// Check all intersecting cells of a round-ended corridor. Only known
// zero-cost cells qualify; inflation, unknown space and out-of-map disable cruise.
// This is an eligibility check, not a substitute for Nav2 collision checking.
inline bool clearCorridor(
  const CruiseState & s, const GridView & g, double length, double half_width)
{
  if (!g.cells || g.width == 0 || g.height == 0 ||
    !std::isfinite(g.resolution) || g.resolution <= 0.0 ||
    !std::isfinite(g.origin_x) || !std::isfinite(g.origin_y)) {return false;}
  const double c = std::cos(s.yaw), sn = std::sin(s.yaw);
  const double padding = g.resolution * std::sqrt(2.0) / 2.0;
  const double lateral = half_width + padding;
  const double end_x=s.x+length*c, end_y=s.y+length*sn;
  const double min_x=std::min(s.x,end_x)-lateral, max_x=std::max(s.x,end_x)+lateral;
  const double min_y=std::min(s.y,end_y)-lateral, max_y=std::max(s.y,end_y)+lateral;
  if (min_x < g.origin_x || min_y < g.origin_y ||
    max_x >= g.origin_x + g.width*g.resolution ||
    max_y >= g.origin_y + g.height*g.resolution) {return false;}
  const auto x0 = static_cast<std::size_t>((min_x-g.origin_x)/g.resolution);
  const auto x1 = static_cast<std::size_t>((max_x-g.origin_x)/g.resolution);
  const auto y0 = static_cast<std::size_t>((min_y-g.origin_y)/g.resolution);
  const auto y1 = static_cast<std::size_t>((max_y-g.origin_y)/g.resolution);
  for (std::size_t y = y0; y <= y1; ++y) {
    for (std::size_t x = x0; x <= x1; ++x) {
      const double dx = g.origin_x+(x+0.5)*g.resolution-s.x;
      const double dy = g.origin_y+(y+0.5)*g.resolution-s.y;
      const double along = dx*c+dy*sn, side = -dx*sn+dy*c;
      const double beyond=along-std::clamp(along,0.0,length);
      if (beyond*beyond+side*side <= lateral*lateral &&
        g.cells[y*g.width+x] != 0) {return false;}
    }
  }
  return true;
}

// Return a gradually varying weight, or zero to leave the original critics in
// sole control. Path-end distance also handles Humble, whose CriticData has no goal.
inline double cruiseWeight(
  const CruiseSettings & cfg, const CruiseState & state, const GridView & grid,
  const std::vector<Point> & path, double footprint_radius, double horizon,
  double forward_limit)
{
  if (!valid(cfg) || path.size() < 2 || !std::isfinite(horizon) || horizon <= 0.0 ||
    !std::isfinite(forward_limit) || forward_limit <= 0.0 ||
    !std::isfinite(footprint_radius) || footprint_radius <= 0.0) {return 0.0;}
  for (double v : {state.x,state.y,state.yaw,state.vx,state.wz,state.scan_age}) {
    if (!std::isfinite(v)) {return 0.0;}
  }
  if (state.scan_age < 0.0 || state.scan_age > cfg.scan_timeout ||
    state.vx < -0.01 || std::abs(state.wz) >= 0.15) {return 0.0;}
  const double target = std::min(cfg.target_speed,forward_limit);
  const double lookahead = std::max(1.0,target*horizon);
  const double c = std::cos(state.yaw), sn = std::sin(state.yaw);
  double length = 0.0, max_error = 0.0;
  Point previous{state.x,state.y};
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto p = path[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return 0.0;}
    // Include the full local path, so bends just beyond the prediction horizon
    // release the preference before the robot reaches them.
    const double dx=p.x-previous.x, dy=p.y-previous.y;
    const double distance=std::hypot(dx,dy);
    if (i == 0) {
      if (distance > 0.20) {return 0.0;}
    } else if (distance > 1e-4) {
      const double delta=std::atan2(dy,dx)-state.yaw;
      const double error=std::abs(std::atan2(std::sin(delta),std::cos(delta)));
      max_error=std::max(max_error,error);
      if (max_error >= 0.15) {return 0.0;}
      length+=distance;
    }
    if (std::abs(-(p.x-state.x)*sn+(p.y-state.y)*c) > 0.15) {return 0.0;}
    previous=p;
  }
  const double end_distance=std::hypot(path.back().x-state.x,path.back().y-state.y);
  const double available=std::min(length,end_distance);
  if (available <= lookahead) {return 0.0;}
  if (!clearCorridor(state,grid,lookahead,footprint_radius+cfg.clearance_margin+0.15)) {
    return 0.0;
  }
  const double path_factor=std::clamp((available-lookahead)/0.5,0.0,1.0);
  const double heading_factor=1.0-max_error/0.15;
  const double turning_factor=1.0-std::abs(state.wz)/0.15;
  return cfg.cost_weight*path_factor*heading_factor*turning_factor;
}

inline double speedPenalty(double speed, double target)
{
  if (!std::isfinite(speed)) {return 1.0;}
  const double error=(target-speed)/target;
  return std::min(error*error,1.0);
}
}  // namespace my_nav2_config
