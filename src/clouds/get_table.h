#pragma once
#include <vector>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "utils_pcl.h"

void getTable(pcl::PointCloud<ColorPoint>::Ptr cloud, std::vector<Eigen::Vector3f>& corners, Eigen::Vector3f& normals, int ind=0);
