#pragma once
#include "ml_scan_context/types.hpp"

namespace ml_scan_context {
// 不可变比较缓存，仅由PolarContext生成，避免调用者修改后失配。
class PreparedDescriptor {
  friend class PolarContext;
  Descriptor normalized_;
  std::vector<unsigned char> occupied_;
};

class PolarContext {
public:
  explicit PolarContext(PolarConfig config = {});
  // 输入已预处理点云；本层不裁剪、不下采样、不增加安装高度。
  Descriptor describe(const Points & points, const Eigen::Vector2d & viewpoint = Eigen::Vector2d::Zero()) const;
  Alignment compare(const Descriptor & query, const Descriptor & target) const;
  PreparedDescriptor prepare(const Descriptor & descriptor) const;
  Alignment comparePrepared(const PreparedDescriptor & query, const PreparedDescriptor & target) const;
  const PolarConfig & config() const noexcept { return config_; }
private:
  PolarConfig config_;
};
}  // namespace ml_scan_context
