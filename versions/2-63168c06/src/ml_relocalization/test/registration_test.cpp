#include <gtest/gtest.h>
#include "registration.hpp"
#include <pcl/common/transforms.h>
#include <random>
namespace ml_relocalization {
TEST(NdtRegistration, ParameterAndInputValidation) {
  Config c; EXPECT_NO_THROW(ValidateRegistrationConfig(c));
  c.ndt_resolution=0; EXPECT_THROW(ValidateRegistrationConfig(c),std::invalid_argument);
  c=Config{}; c.ndt_threads=0; EXPECT_THROW(ValidateRegistrationConfig(c),std::invalid_argument);
  c=Config{}; c.inlier_fraction_threshold=1.1; EXPECT_THROW(ValidateRegistrationConfig(c),std::invalid_argument);
  c=Config{}; c.ndt_likelihood_threshold=NAN; EXPECT_THROW(ValidateRegistrationConfig(c),std::invalid_argument);
  MatchResult r; auto empty=std::make_shared<Cloud>();
  refine(Config{},empty,empty,Eigen::Matrix4d::Identity(),r);
  EXPECT_FALSE(r.valid); EXPECT_FALSE(r.registration_ran);
}
TEST(NdtRegistration, AbsolutePoseAndQualityRejection) {
  auto target=std::make_shared<Cloud>(); std::mt19937 gen(41); std::normal_distribution<float> noise(0,.18);
  for(int x=0;x<8;++x) for(int y=0;y<7;++y) for(int j=0;j<180;++j) {
    Point p;p.x=2.5f*x+noise(gen);p.y=2.5f*y+noise(gen);p.z=.45f*x+.13f*y*y+noise(gen);p.intensity=1;target->push_back(p);
  }
  Eigen::Matrix4d truth=Eigen::Matrix4d::Identity();
  truth.block<3,3>(0,0)=(Eigen::AngleAxisd(.23,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(.06,Eigen::Vector3d::UnitY())).toRotationMatrix();
  truth.block<3,1>(0,3)=Eigen::Vector3d(3,-2,.7);
  auto source=std::make_shared<Cloud>();pcl::transformPointCloud(*target,*source,truth.inverse().eval());
  auto initial=truth;initial(0,3)+=.12;Config c;MatchResult r;
  refine(c,source,target,initial,r);
  EXPECT_TRUE(r.valid)<<r.reason;EXPECT_GT(r.likelihood,2.3);EXPECT_GT(r.inlier_fraction,.99);
  EXPECT_LT((r.T_map_body-truth).norm(),.12);EXPECT_LT(r.correction_m,.3);
  c.ndt_likelihood_threshold=1e6;MatchResult reject;refine(c,source,target,initial,reject);
  EXPECT_FALSE(reject.valid);EXPECT_EQ(reject.reason,"ndt_likelihood");
  initial(0,0)=NAN;MatchResult invalid;refine(c,source,target,initial,invalid);
  EXPECT_FALSE(invalid.valid);EXPECT_FALSE(invalid.registration_ran);
}
} // namespace ml_relocalization
