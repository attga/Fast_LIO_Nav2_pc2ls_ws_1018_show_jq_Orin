#include "sc_pgo/loop_snapshot.h"
#include <stdexcept>
#include <iostream>
void require(bool ok) { if (!ok) throw std::runtime_error("回环快照测试失败"); }
int main() {
    std::vector<pcl::PointCloud<PointType>::Ptr> clouds;
    std::vector<Pose6D> poses;
    std::vector<double> times;
    for(int i=0;i<4;++i) {
        clouds.emplace_back(new pcl::PointCloud<PointType>());
        clouds.back()->resize(1); clouds.back()->front().x=i;
        poses.push_back(Pose6D{double(i),0,0,0,0,0}); times.push_back(100+i);
    }
    auto snapshot=sc_pgo::captureLoopSnapshot(clouds,poses,times,0,3,1,1);
    require(bool(snapshot)); require(snapshot->source.size()==2 && snapshot->target.size()==2);
    require(snapshot->source.front().pose.x==2 && snapshot->target.back().pose.x==1);
    // 模拟配准期间另一线程更新位姿并释放原索引；快照仍保持原几何和点云生命期。
    poses[0].x=100;poses[3].x=103;clouds.clear();poses.clear();times.clear();
    require(snapshot->history_pose.x==0 && snapshot->current_pose.x==3 && snapshot->scan_time==103);
    require(snapshot->source.back().pose.x==3 && snapshot->source.back().cloud->front().x==3);
    require(!sc_pgo::captureLoopSnapshot(clouds,poses,times,0,1));
    poses.push_back(Pose6D{});require(!sc_pgo::captureLoopSnapshot(clouds,poses,times,0,0));
    std::cout<<"快照范围、位姿隔离、只读点云生命期与无效索引测试通过\n";
}
