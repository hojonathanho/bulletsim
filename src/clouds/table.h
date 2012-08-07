#pragma once
#include "utils_pcl.h"
#include <Eigen/Dense>
#include <pcl/Vertices.h>

ColorCloudPtr getPointsOnTableHull(ColorCloudPtr cloud, ColorCloudPtr hull, std::vector<pcl::Vertices> polys, float height);
float getTableHeight(ColorCloudPtr cloud);
ColorCloudPtr getTablePoints(ColorCloudPtr cloud, float height);
void getTableXYBounds(ColorCloudPtr, float&, float&, float&, float&);
