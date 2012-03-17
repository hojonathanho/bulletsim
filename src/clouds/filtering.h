#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include "clouds/utils_pcl.h"


ColorCloudPtr maskCloud(const ColorCloudPtr in, const cv::Mat& mask);
ColorCloudPtr maskCloud(const ColorCloudPtr in, const VectorXb& mask);
ColorCloudPtr downsampleCloud(const ColorCloudPtr in, float voxelSizeMeters);
ColorCloudPtr removeOutliers(const ColorCloudPtr in, float thresh=1, int k=15);
ColorCloudPtr removeZRange(const ColorCloudPtr in, float minZ, float maxZ);
void labelCloud(ColorCloudPtr in, const cv::Mat& labels);
