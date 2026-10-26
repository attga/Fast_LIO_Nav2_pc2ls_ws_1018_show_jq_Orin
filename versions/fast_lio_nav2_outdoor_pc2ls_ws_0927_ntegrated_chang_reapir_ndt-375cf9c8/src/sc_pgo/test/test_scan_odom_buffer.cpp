#include "sc_pgo/scan_odom_buffer.h"
#include <iostream>
#include <stdexcept>
void require(bool ok) { if (!ok) throw std::runtime_error("配对测试失败"); }
int main() {
  using Buffer = sc_pgo::ScanOdomBuffer<int,int>;
  Buffer b(2);
  // 跨话题异步到达，仍只取相同纳秒时间的消息。
  b.pushScan(100,10); require(!b.take()); b.pushOdom(100,20);
  auto p=b.take(); require(p && p->first==20 && p->second==10); require(!b.take());
  // 未来位姿不能配旧扫描；相差一纳秒也不允许静默错配。
  b.pushScan(200,1); b.pushOdom(201,2); require(!b.take());
  require(b.stats().scan_unmatched==1); b.pushScan(201,3); require(bool(b.take()));
  b.pushOdom(300,4); b.pushScan(400,5); require(!b.take());
  require(b.stats().odom_unmatched==1); b.pushOdom(400,6); require(bool(b.take()));
  // 重复、倒序输入不可导致重复处理或时间倒退。
  b.pushOdom(400,7); b.pushOdom(399,8); require(b.stats().odom_nonmonotonic==2);
  b.pushScan(400,7); b.pushScan(399,8); require(b.stats().scan_nonmonotonic==2);
  // 缺失另一话题时限制缓存，恢复后只配保留的数据。
  b.pushOdom(500,1);b.pushOdom(600,2);b.pushOdom(700,3);
  require(b.pendingOdom()==2 && b.stats().odom_overflow==1);
  b.pushScan(500,1);b.pushScan(600,2);b.pushScan(700,3);
  require(b.pendingScan()==2 && b.stats().scan_overflow==1);
  require(bool(b.take()));require(bool(b.take()));require(!b.take());
  require(b.stats().matched==5);
  bool rejected=false;try { Buffer invalid(0); } catch(const std::invalid_argument&) { rejected=true; }
  require(rejected);std::cout<<"严格配对、丢帧、乱序、重复、缓存上限测试通过\n";
}
