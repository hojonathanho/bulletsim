#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "utils_pcl.h"



std::vector<float> getPlaneCoeffsRansac(ColorCloudPtr cloud);

std::vector<int> getPlaneInliers(ColorCloudPtr cloud, std::vector<float> coeffs);
