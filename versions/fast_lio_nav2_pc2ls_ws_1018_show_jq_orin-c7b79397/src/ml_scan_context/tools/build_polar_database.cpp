// 离线建库入口：PCD依赖留在工具边界，描述子核心不依赖PCL或ROS。
#include "ml_scan_context/database.hpp"
#include "ml_scan_context/preprocessing.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <chrono>
#include <iostream>
#include <map>
#include <stdexcept>

namespace fs = std::filesystem;
using namespace ml_scan_context;

int main(int argc, char ** argv)
{
  try {
    if (argc != 4) {
      std::cerr << "Usage: build_polar_database SCANS_DIRECTORY OUTPUT.mlpc FRAME_CONVENTION\n"
                << "Preset: XY < 60 m, height +2 m, floor-centroid voxel 0.5 m; "
                << "20x60 bins, radius 80 m, minimum 5 points, 7 viewpoints.\n"
                << "FRAME_CONVENTION is a label, not a coordinate transform.\n";
      return 2;
    }
    const fs::path scans(argv[1]), output(argv[2]);
    if (output.extension() != ".mlpc" || fs::exists(output) || fs::is_symlink(output)) {
      throw std::runtime_error("Output must be a new .mlpc file; overwriting is forbidden");
    }
    std::map<KeyframeId, fs::path> files;
    for (const auto & entry : fs::directory_iterator(scans)) {
      if (!entry.is_regular_file() || entry.path().extension() != ".pcd") continue;
      const auto name = entry.path().stem().string();
      if (name.empty() || name.find_first_not_of("0123456789") != std::string::npos) {
        throw std::runtime_error("Non-numeric keyframe filename: " + entry.path().string());
      }
      const auto id = std::stoull(name);
      if (!files.emplace(id, entry.path()).second) {
        throw std::runtime_error("Duplicate numeric keyframe ID: " + name);
      }
    }
    if (files.empty()) throw std::runtime_error("No keyframe PCD files found");
    // 保留文件名中的真实ID，允许稀疏ID；消费者负责与位姿库核对。
    Preprocessor preprocessing;
    DatabaseConfig config;
    config.preprocessing_id = preprocessing.identifier(argv[3]);
    Database database(config);
    const auto start = std::chrono::steady_clock::now();
    std::size_t raw_count = 0, prepared_count = 0;
    for (const auto & [id, path] : files) {
      pcl::PCLPointCloud2 blob;
      if (pcl::io::loadPCDFile(path.string(), blob) < 0) {
        throw std::runtime_error("Cannot read PCD: " + path.string());
      }
      for (const auto * field : {"x", "y", "z"}) {
        const int index = pcl::getFieldIndex(blob, field);
        if (index < 0 || blob.fields[index].datatype != pcl::PCLPointField::FLOAT32 ||
          blob.fields[index].count != 1)
        {
          throw std::runtime_error("PCD requires scalar float32 XYZ: " + path.string());
        }
      }
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromPCLPointCloud2(blob, cloud);
      Points raw;
      raw.reserve(cloud.size());
      for (const auto & p : cloud) raw.emplace_back(p.x, p.y, p.z);
      auto prepared = preprocessing.apply(raw);
      if (prepared.empty()) throw std::runtime_error("Empty prepared keyframe: " + path.string());
      raw_count += raw.size();
      prepared_count += prepared.size();
      database.add(id, prepared);
      if (database.size() % 250 == 0) std::cout << "built " << database.size() << '/' << files.size() << std::endl;
    }
    database.save(output);
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::cout << "saved=" << output << " keyframes=" << database.size()
              << " raw_points=" << raw_count << " prepared_points=" << prepared_count
              << " elapsed_s=" << elapsed << " bytes=" << fs::file_size(output)
              << "\npreprocessing=" << config.preprocessing_id << '\n';
  } catch (const std::exception & error) {
    std::cerr << "build_polar_database: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
