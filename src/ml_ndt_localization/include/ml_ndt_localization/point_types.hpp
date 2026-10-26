#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ml_ndt_loc
{
using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;
} // namespace ml_ndt_loc
