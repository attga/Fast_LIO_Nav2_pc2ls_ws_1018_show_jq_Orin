#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <pcl/io/pcd_io.h>
#include "ml_relocalization/relocalizer.hpp"
#include "ml_scan_context/database.hpp"
#include "ml_scan_context/preprocessing.hpp"

namespace {
class PolarContract : public ::testing::Test {
protected:
  std::filesystem::path directory;
  ml_relocalization::Config config;
  void SetUp() override {
    char pattern[] = "/tmp/polar_contract_XXXXXX";
    const auto path = mkdtemp(pattern);
    ASSERT_NE(path,nullptr);directory=path;
    std::filesystem::create_directory(directory/"Scans");
    std::ofstream(directory/"isam_poses.txt") << "1 0 0 0 0 1 0 0 0 0 1 0\n";
    ml_relocalization::Cloud cloud;ml_relocalization::Point p;p.x=1;p.y=0;p.z=0;p.intensity=1;cloud.push_back(p);
    pcl::io::savePCDFileBinary((directory/"Scans/000000.pcd").string(),cloud);
    config.map_directory=directory.string();config.descriptor_backend="polar";
    config.polar_database_file=(directory/"map.mlpc").string();
  }
  void TearDown() override { if(!directory.empty())std::filesystem::remove_all(directory); }
  void database(std::uint64_t id) {
    ml_scan_context::DatabaseConfig c;
    c.preprocessing_id=ml_scan_context::Preprocessor{}.identifier("livox_imu");
    ml_scan_context::Database db(c);db.add(id,{});db.save(config.polar_database_file);
  }
};
TEST_F(PolarContract, RejectsWrongIdsEvenWhenCountsMatch) {
  database(7);EXPECT_THROW(ml_relocalization::Relocalizer r(config),std::runtime_error);
}
TEST_F(PolarContract, RequiresIndependentRawDescriptorInput) {
  database(0);ml_relocalization::Relocalizer r(config);
  auto cloud=std::make_shared<ml_relocalization::Cloud>();ml_relocalization::Point p;p.x=1;p.y=0;p.z=0;cloud->push_back(p);
  EXPECT_EQ(r.match(cloud).reason,"polar_raw_input_missing");
  EXPECT_EQ(r.match(nullptr).reason,"registration_input_empty");
  EXPECT_EQ(r.match(cloud,cloud).reason,"polar_no_candidate");
}
TEST_F(PolarContract, RejectsMismatchedPreprocessingLabel) {
  database(0);config.polar_frame_convention="different_frame";
  EXPECT_THROW(ml_relocalization::Relocalizer r(config),std::runtime_error);
}
}
