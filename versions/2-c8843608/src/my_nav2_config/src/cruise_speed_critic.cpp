#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "my_nav2_config/cruise_policy.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mppi::critics
{
class CruiseSpeedCritic : public CriticFunction
{
public:
  void initialize() override
  {
    auto get = parameters_handler_->getParamGetter(name_);
    get(settings_.target_speed,"target_speed",0.8,ParameterType::Static);
    get(settings_.cost_weight,"cost_weight",8.0,ParameterType::Static);
    get(settings_.clearance_margin,"clearance_margin",0.15,ParameterType::Static);
    get(settings_.scan_timeout,"scan_timeout",0.3,ParameterType::Static);
    std::string scan_topic;
    get(scan_topic,"scan_topic",std::string("/scan"),ParameterType::Static);
    auto parent_get = parameters_handler_->getParamGetter(parent_name_);
    parent_get(forward_limit_,"vx_max",0.8f);
    if (!my_nav2_config::valid(settings_) || scan_topic.empty()) {
      throw std::invalid_argument("Invalid CruiseSpeedCritic configuration");
    }
    // The callback owns only the shared atomic, so it cannot access a destroyed critic.
    scan_stamp_ = std::make_shared<std::atomic<std::int64_t>>(0);
    scan_sub_ = parent_.lock()->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic,rclcpp::SensorDataQoS(),
      [stamp=scan_stamp_](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        if (msg->ranges.empty() || !std::isfinite(msg->range_min) ||
          !std::isfinite(msg->range_max) || msg->range_max <= msg->range_min ||
          !std::isfinite(msg->angle_increment) || msg->angle_increment <= 0.0 ||
          msg->header.stamp.nanosec >= 1000000000U) {stamp->store(0); return;}
        bool valid_range=false;
        for (float range : msg->ranges) {
          if ((std::isfinite(range) && range >= msg->range_min && range <= msg->range_max) ||
            (std::isinf(range) && range > 0.0f)) {valid_range=true; break;}
        }
        if (!valid_range) {stamp->store(0); return;}
        stamp->store(static_cast<std::int64_t>(msg->header.stamp.sec)*1000000000LL+
          msg->header.stamp.nanosec);
      });
    RCLCPP_INFO(logger_,"Cruise speed preference %.2f m/s on clear straight paths",
      settings_.target_speed);
  }

  void score(CriticData & data) override
  {
    if (!enabled_ || data.fail_flag || !costmap_ros_->isCurrent()) {return;}
    const auto stamp=scan_stamp_->load();
    if (stamp <= 0) {return;}
    auto node=parent_.lock();
    if (!node) {return;}
    const double scan_age=(static_cast<double>(node->now().nanoseconds())-
      static_cast<double>(stamp))*1e-9;
    const auto & pose=data.state.pose.pose;
    const auto & q=pose.orientation;
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if (!std::isfinite(norm) || std::abs(norm-1.0) > 0.01) {return;}
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    const auto rows=data.state.vx.shape(0), steps=data.state.vx.shape(1);
    if (steps == 0 || data.costs.shape(0) != rows ||
      data.path.x.shape(0) != data.path.y.shape(0)) {return;}
    std::vector<my_nav2_config::Point> path;
    path.reserve(data.path.x.shape(0));
    for (std::size_t i=0;i<data.path.x.shape(0);++i) {
      path.push_back({data.path.x(i),data.path.y(i)});
    }
    double radius=0.0;
    for (const auto & p : costmap_ros_->getRobotFootprint()) {
      const double r=std::hypot(p.x,p.y);
      if (!std::isfinite(r)) {return;}
      radius=std::max(radius,r);
    }
    const my_nav2_config::CruiseState state{pose.position.x,pose.position.y,yaw,
      data.state.speed.linear.x,data.state.speed.angular.z,scan_age};
    double weight=0.0;
    {
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
      const my_nav2_config::GridView grid{costmap_->getCharMap(),
        costmap_->getSizeInCellsX(),costmap_->getSizeInCellsY(),costmap_->getResolution(),
        costmap_->getOriginX(),costmap_->getOriginY()};
      weight=my_nav2_config::cruiseWeight(settings_,state,grid,path,radius,
        steps*static_cast<double>(data.model_dt),forward_limit_);
    }
    if (weight <= 0.0) {return;}
    const double target=std::min(settings_.target_speed,static_cast<double>(forward_limit_));
    for (std::size_t row=0;row<rows;++row) {
      double penalty=0.0;
      for (std::size_t step=0;step<steps;++step) {
        penalty+=my_nav2_config::speedPenalty(data.state.vx(row,step),target);
      }
      // Bounded soft preference; never changes commands, constraints or other costs.
      data.costs(row)+=static_cast<float>(weight*penalty/steps);
    }
  }

private:
  my_nav2_config::CruiseSettings settings_;
  float forward_limit_{0.8f};
  std::shared_ptr<std::atomic<std::int64_t>> scan_stamp_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};
}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::CruiseSpeedCritic,mppi::critics::CriticFunction)
