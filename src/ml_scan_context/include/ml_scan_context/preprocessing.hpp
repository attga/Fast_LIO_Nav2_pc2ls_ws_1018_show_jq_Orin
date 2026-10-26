#pragma once
#include "ml_scan_context/types.hpp"

namespace ml_scan_context {
struct PreprocessingConfig {
  double xy_radius = 60.0;
  double height_offset = 2.0;
  double voxel_size = 0.5;
};

// 原点固定的floor体素、质心输出、体素键字典序；输入须已转到描述子坐标系。
// 顺序：有限点筛选 → XY严格半径裁剪 → Z偏移 → 体素质心。
class Preprocessor {
public:
  explicit Preprocessor(PreprocessingConfig config = {});
  Points apply(const Points & points) const;
  // 包含数值参数和算法版本；调用者传入坐标语义标签，不执行TF变换。
  std::string identifier(const std::string & frame_convention) const;
private:
  PreprocessingConfig config_;
};
}  // namespace ml_scan_context
