#pragma once
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <utility>

namespace sc_pgo {
// 调用方负责加锁。同一前端的点云和位姿严格按整数纳秒配对，不插值。
template<class Odom, class Scan>
class ScanOdomBuffer {
 public:
  struct Stats {
    std::uint64_t matched = 0, odom_unmatched = 0, scan_unmatched = 0;
    std::uint64_t odom_overflow = 0, scan_overflow = 0;
    std::uint64_t odom_nonmonotonic = 0, scan_nonmonotonic = 0;
  };
  explicit ScanOdomBuffer(std::size_t limit = 100) : limit_(limit) {
    if (!limit) throw std::invalid_argument("input_queue_size 必须大于零");
  }
  void pushOdom(std::int64_t stamp, Odom value) {
    push(odom_, last_odom_, stamp, std::move(value), stats_.odom_overflow, stats_.odom_nonmonotonic);
  }
  void pushScan(std::int64_t stamp, Scan value) {
    push(scan_, last_scan_, stamp, std::move(value), stats_.scan_overflow, stats_.scan_nonmonotonic);
  }
  std::optional<std::pair<Odom, Scan>> take() {
    while (!odom_.empty() && !scan_.empty()) {
      if (odom_.front().first < scan_.front().first) {
        odom_.pop_front(); ++stats_.odom_unmatched;
      } else if (scan_.front().first < odom_.front().first) {
        scan_.pop_front(); ++stats_.scan_unmatched;
      } else {
        auto result = std::make_pair(std::move(odom_.front().second), std::move(scan_.front().second));
        odom_.pop_front(); scan_.pop_front(); ++stats_.matched;
        return result;
      }
    }
    return std::nullopt;
  }
  const Stats& stats() const { return stats_; }
  std::size_t pendingOdom() const { return odom_.size(); }
  std::size_t pendingScan() const { return scan_.size(); }
 private:
  template<class T>
  void push(std::deque<std::pair<std::int64_t,T>>& queue, std::optional<std::int64_t>& last,
            std::int64_t stamp, T value, std::uint64_t& overflow, std::uint64_t& nonmonotonic) {
    // 单话题保持单调；跨话题到达先后不同不影响配对。
    if (last && stamp <= *last) { ++nonmonotonic; return; }
    last = stamp;
    if (queue.size() == limit_) { queue.pop_front(); ++overflow; }
    queue.emplace_back(stamp, std::move(value));
  }
  std::size_t limit_;
  Stats stats_;
  std::optional<std::int64_t> last_odom_, last_scan_;
  std::deque<std::pair<std::int64_t,Odom>> odom_;
  std::deque<std::pair<std::int64_t,Scan>> scan_;
};
}
