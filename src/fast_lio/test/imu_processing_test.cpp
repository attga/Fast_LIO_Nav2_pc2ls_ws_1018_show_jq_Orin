#include <gtest/gtest.h>
#include <omp.h>
#include "IMU_Processing.hpp"

namespace {
using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;
MeasureGroup measurement(double begin) {
    MeasureGroup group;
    group.lidar_beg_time=begin;
    group.lidar_end_time=begin+0.1;
    group.lidar->resize(2);
    (*group.lidar)[0].x=1.; (*group.lidar)[0].curvature=0.;
    (*group.lidar)[1].x=2.; (*group.lidar)[1].curvature=100.;
    for (int i=1; i<=20; ++i) {
        auto imu=std::make_shared<sensor_msgs::msg::Imu>();
        imu->header.stamp=rclcpp::Time(static_cast<int64_t>((begin+i*0.005)*1e9));
        imu->linear_acceleration.z=G_m_s2;
        group.imu.push_back(imu);
    }
    return group;
}
TEST(ImuProcessing, MissingInputClearsOutputAndPreservesState) {
    ImuProcess process;
    Filter filter;
    auto output=std::make_shared<PointCloudXYZI>();
    output->resize(2);
    const auto before=filter.get_x().pos;
    auto group=measurement(1.0);
    group.imu.clear();
    EXPECT_FALSE(process.Process(group,filter,output));
    EXPECT_TRUE(output->empty());
    EXPECT_TRUE(filter.get_x().pos.isApprox(before));
    group=measurement(1.0);
    group.lidar.reset();
    output->resize(2);
    EXPECT_FALSE(process.Process(group,filter,output));
    EXPECT_TRUE(output->empty());
}
TEST(ImuProcessing, InitializationThenValidScanThenImuGap) {
    ImuProcess process;
    process.log_directory="/dev/null"; // 禁止测试写入前端默认日志目录。
    process.set_gyr_cov(V3D::Constant(.1));
    process.set_acc_cov(V3D::Constant(.1));
    Filter filter;
    double epsilon[23]; std::fill(epsilon,epsilon+23,.001);
    filter.init_dyn_share(get_f,df_dx,df_dw,
        [](state_ikfom&,esekfom::dyn_share_datastruct<double>&) {},4,epsilon);
    auto output=std::make_shared<PointCloudXYZI>();
    output->resize(2);
    EXPECT_FALSE(process.Process(measurement(1.0),filter,output));
    EXPECT_TRUE(output->empty());
    ASSERT_TRUE(process.Process(measurement(1.1),filter,output));
    ASSERT_EQ(output->size(),2u);
    const auto before=filter.get_x().pos;
    auto gap=measurement(1.2); gap.imu.clear();
    EXPECT_FALSE(process.Process(gap,filter,output));
    EXPECT_TRUE(output->empty());
    EXPECT_TRUE(filter.get_x().pos.isApprox(before));
}
}
