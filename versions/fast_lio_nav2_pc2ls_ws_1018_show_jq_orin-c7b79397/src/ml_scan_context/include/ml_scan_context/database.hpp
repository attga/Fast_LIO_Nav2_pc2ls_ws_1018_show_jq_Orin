#pragma once
#include "ml_scan_context/polar_context.hpp"
#include <filesystem>

namespace ml_scan_context {
// 不含ROS、全局位姿或配准器。并发查询只读安全，写入由调用者串行化。
class Database {
public:
  explicit Database(DatabaseConfig config);
  void add(KeyframeId id, const Points & points);
  std::vector<Candidate> query(const Points & points, const QueryOptions & options = {}) const;
  std::size_t size() const noexcept { return frames_.size(); }
  const DatabaseConfig & config() const noexcept { return config_; }
  std::vector<KeyframeId> keyframeIds() const;
  // 新格式MLPCDB，与旧sc_db.bin无关。加载失败不改变当前数据库。
  void save(const std::filesystem::path & path) const;
  void load(const std::filesystem::path & path);
private:
  struct Frame {
    KeyframeId id;
    std::vector<Descriptor> descriptors;
    std::vector<PreparedDescriptor> prepared;
  };
  DatabaseConfig config_;
  PolarContext context_;
  std::vector<Frame> frames_;
};
}  // namespace ml_scan_context
