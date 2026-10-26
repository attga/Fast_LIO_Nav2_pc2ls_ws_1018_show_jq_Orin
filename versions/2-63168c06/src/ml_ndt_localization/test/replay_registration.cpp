// 离线回归工具：复用生产适配层，输入与历史公平 A/B 清单相同。不会发布 ROS 接口。
#include "ml_ndt_localization/ndt_matcher.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
using namespace ml_ndt_loc;
using Clock=std::chrono::steady_clock;
PointCloud::Ptr Downsample(PointCloud::Ptr cloud) {
    auto result=pcl::make_shared<PointCloud>();
    pcl::VoxelGrid<PointType> filter; filter.setLeafSize(.4,.4,.4);
    filter.setInputCloud(cloud); filter.filter(*result); return result;
}
int main(int argc,char** argv) {
    if(argc!=3) {std::cerr<<"replay_registration MAP INPUTS\n";return 2;}
    auto raw=pcl::make_shared<PointCloud>();
    if(pcl::io::loadPCDFile(argv[1],*raw)) return 3;
    auto map=Downsample(raw);
    PointCloud::Ptr target;
    NdtMatcher matcher(NdtMatcher::Config{});
    double last_x=NAN,last_y=NAN;
    std::ifstream inputs(argv[2]);
    std::string id,path; double stamp,x,y,z,yaw,cx,cy;
    std::cout<<"case_id,timestamp,initial_x,initial_y,initial_z,initial_yaw,result_x,result_y,result_z,result_yaw,nvtl,transform_probability,converged,iterations,latency_ms,target_ms,total_ms,cpu_ms,inlier1,correction_translation,correction_yaw,accepted,scan_points,map_points";
    for(int i=0;i<16;++i)std::cout<<",T"<<i;
    std::cout<<",covariance_xx,covariance_xy,covariance_yy,covariance_valid,covariance_ms"<<std::endl;
    while(inputs>>id>>stamp>>path>>x>>y>>z>>yaw>>cx>>cy) {
        const auto begin=Clock::now(); const auto cpu=std::clock();
        if(cx!=last_x||cy!=last_y) {
            target=pcl::make_shared<PointCloud>();
            for(const auto& p:*map)if(std::abs(p.x-cx)<=100&&std::abs(p.y-cy)<=100)target->push_back(p);
            last_x=cx;last_y=cy;
        }
        auto scan=pcl::make_shared<PointCloud>();
        if(pcl::io::loadPCDFile(path,*scan))return 4;
        scan=Downsample(scan);
        Eigen::Matrix4d initial=Eigen::Matrix4d::Identity();
        // 与冻结 benchmark 相同的 float 初值，避免把量化差异误报为算法变化。
        Eigen::Matrix4f init_f=pcl::getTransformation(float(x),float(y),float(z),0.f,0.f,float(yaw*M_PI/180)).matrix();
        initial=init_f.cast<double>();
        auto r=matcher.Align(scan,target,initial);
        auto& t=r.T_map_base;
        double dy=std::remainder(std::atan2(t(1,0),t(0,0))*180/M_PI-yaw,360.);
        double correction=(t.block<3,1>(0,3)-initial.block<3,1>(0,3)).norm();
        bool accepted=r.covariance_valid && r.converged && r.likelihood>=2.3 && r.inlier_fraction>=.4 && correction<=2 && std::abs(dy)<=15;
        std::cout<<std::setprecision(12)<<id<<","<<stamp<<","<<x<<","<<y<<","<<z<<","<<yaw<<","<<t(0,3)<<","<<t(1,3)<<","<<t(2,3)<<","<<std::atan2(t(1,0),t(0,0))*180/M_PI<<","<<r.likelihood<<","<<r.transformation_probability<<","<<r.converged<<","<<r.iterations<<","<<r.registration_ms<<","<<r.target_build_ms<<","<<std::chrono::duration<double,std::milli>(Clock::now()-begin).count()<<","<<1000.*(std::clock()-cpu)/CLOCKS_PER_SEC<<","<<r.inlier_fraction<<","<<correction<<","<<dy<<","<<accepted<<","<<scan->size()<<","<<target->size();
        for(int i=0;i<16;++i)std::cout<<","<<t(i/4,i%4);
        std::cout<<","<<r.covariance_xy(0,0)+.09<<","<<r.covariance_xy(0,1)<<","<<r.covariance_xy(1,1)+.09<<","<<r.covariance_valid<<","<<r.covariance_ms<<std::endl;
    }
}
