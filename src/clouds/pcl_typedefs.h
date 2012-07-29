#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<ColorPoint> ColorCloud;
typedef pcl::PointCloud<ColorPoint>::ConstPtr ConstColorCloudPtr;
typedef pcl::PointCloud<ColorPoint>::Ptr ColorCloudPtr;

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef pcl::PointCloud<Point>::ConstPtr ConstCloudPtr;
typedef pcl::PointCloud<Point>::Ptr CloudPtr;
