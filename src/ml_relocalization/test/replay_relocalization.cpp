// 同一冻结扫描/SC/关键帧子图的离线回归，不发布 ROS 接口。
#include "ml_relocalization/relocalizer.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
using namespace ml_relocalization;
int main(int argc,char**argv) {
  if(argc!=3 && argc!=4)return 2;
  Config c;c.map_directory=argv[1];c.sc_dist_thres=.5;c.confirm_frames=2;
  if(argc==4) {c.descriptor_backend="polar";c.polar_database_file=argv[3];}
  Relocalizer reloc(c);std::ifstream file(argv[2]);std::string id,ds,path;double stamp;
  std::cout<<"case_id,dataset,timestamp,candidate,sc_distance,registration_ran,converged,iterations,nvtl,inlier1,correction_m,correction_yaw_deg,accepted,reason,attempt_ms";
  std::cout<<",candidates_tested,candidates_accepted";
  for(int i=0;i<16;++i)std::cout<<",T"<<i;std::cout<<std::endl;
  while(file>>id>>ds>>stamp>>path) {
    auto raw=std::make_shared<Cloud>();auto scan=std::make_shared<Cloud>();
    if(pcl::io::loadPCDFile(path,*raw))return 3;
    pcl::VoxelGrid<Point> v;v.setInputCloud(raw);v.setLeafSize(.4,.4,.4);v.filter(*scan);
    auto begin=std::chrono::steady_clock::now();auto r=reloc.match(scan, raw);
    auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    std::cout<<std::setprecision(12)<<id<<","<<ds<<","<<stamp<<","<<r.candidate<<","<<r.sc_distance<<","<<r.registration_ran<<","<<r.converged<<","<<r.iterations<<","<<r.likelihood<<","<<r.inlier_fraction<<","<<r.correction_m<<","<<r.correction_yaw_deg<<","<<r.valid<<","<<r.reason<<","<<ms;
    std::cout<<","<<r.candidates_tested<<","<<r.candidates_accepted;
    for(int i=0;i<16;++i)std::cout<<","<<r.T_map_body(i/4,i%4);std::cout<<std::endl;
  }
}
