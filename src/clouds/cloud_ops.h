#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "utils_pcl.h"
#include <pcl/Vertices.h>

std::vector< std::vector<int> > findClusters(ColorCloudPtr cloud, float tol=.02, float minSize=100);
ColorCloudPtr downsampleCloud(const ColorCloudPtr in, float sz);
ColorCloudPtr removeOutliers(const ColorCloudPtr in, float thresh, int k);
ColorCloudPtr findConcaveHull(ColorCloudPtr in, std::vector<pcl::Vertices>& polygons);
ColorCloudPtr findConvexHull(ColorCloudPtr in, std::vector<pcl::Vertices>& polygons);
ColorCloudPtr cropToHull(const ColorCloudPtr in, ColorCloudPtr hull, std::vector<pcl::Vertices>& polygons);
ColorCloudPtr projectOntoPlane(const ColorCloudPtr in, Eigen::Vector4f& coeffs);
ColorCloudPtr filterHeight(ColorCloudPtr in, float low, float high);
void fixZ(ColorCloudPtr in, float z);
Eigen::VectorXf getCircle(ColorCloudPtr cloud);
Eigen::VectorXf getEnclosingCircle(ColorCloudPtr cloud);
ColorCloudPtr getBiggestCluster(ColorCloudPtr in, float tol);
