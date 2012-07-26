#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "utils_pcl.h"
#include <pcl/Vertices.h>
#include <opencv2/core/core.hpp>


std::vector< std::vector<int> > findClusters(ColorCloudPtr cloud, float tol=.02, float minSize=100);
ColorCloudPtr downsampleCloud(const ColorCloudPtr in, float sz);
ColorCloudPtr removeOutliers(const ColorCloudPtr in, float thresh=1, int k=15);
ColorCloudPtr findConcaveHull(ColorCloudPtr in, std::vector<pcl::Vertices>& polygons);
ColorCloudPtr findConvexHull(ColorCloudPtr in, std::vector<pcl::Vertices>& polygons);
ColorCloudPtr cropToHull(const ColorCloudPtr in, ColorCloudPtr hull, std::vector<pcl::Vertices>& polygons);
ColorCloudPtr projectOntoPlane(const ColorCloudPtr in, Eigen::Vector4f& coeffs);
ColorCloudPtr filterZ(ColorCloudPtr in, float low, float high);
ColorCloudPtr filterY(ColorCloudPtr in, float low, float high);
ColorCloudPtr filterX(ColorCloudPtr in, float low, float high);
Eigen::VectorXf getCircle(ColorCloudPtr cloud);
Eigen::VectorXf getEnclosingCircle(ColorCloudPtr cloud);
ColorCloudPtr getBiggestCluster(ColorCloudPtr in, float tol);
ColorCloudPtr clusterFilter(ColorCloudPtr in, float tol, float minSize);
ColorCloudPtr maskCloud(const ColorCloudPtr in, const cv::Mat& mask);
ColorCloudPtr maskCloud(const ColorCloudPtr in, const VectorXb& mask);
ColorCloudPtr removeZRange(const ColorCloudPtr in, float minZ, float maxZ);
void labelCloud(ColorCloudPtr in, const cv::Mat& labels);
ColorCloudPtr hueFilter(ColorCloudPtr in, uint8_t minHue, uint8_t maxHue, uint8_t minSat=0, uint8_t maxSat=255, uint8_t minVal=0, uint8_t maxVal=255);
ColorCloudPtr orientedBoxFilter(ColorCloudPtr cloud_in, const Eigen::Matrix3f& ori, const Eigen::Vector3f& mins, const Eigen::Vector3f& maxes);
ColorCloudPtr boxFilter(ColorCloudPtr cloud_in, const Eigen::Vector3f& mins, const Eigen::Vector3f& maxes);
