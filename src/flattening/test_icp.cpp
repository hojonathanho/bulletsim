// Adapted from http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
// Takes point clouds A and B, aligns A to B, and displays A in yellow and B in gray
// usage: test_icp input.pcd target.pcd

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include "cloudutils.h"

static pcl::PointCloud<pcl::PointXYZRGB>::Ptr convert(ColorCloudPtr in, int r, int g, int b) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
  out->points.resize(in->size());
  for (size_t i = 0; i < in->size(); ++i) {
    out->points[i].x = in->points[i].x;
    out->points[i].y = in->points[i].y;
    out->points[i].z = in->points[i].z;
    out->points[i].r = r;
    out->points[i].g = g;
    out->points[i].b = b;
  }
  return out;
}

int main (int argc, char** argv) {
  ColorCloudPtr in1 = readPCD(argv[1]);
  ColorCloudPtr in2 = readPCD(argv[2]);
  pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
  icp.setInputCloud(in1);
  icp.setInputTarget(in2);
  ColorCloudPtr final(new ColorCloud);
  icp.align(*final);
  cout << "aligned -> target score: " << icp.getFitnessScore() << endl;
  cout << "target -> aligned score: " << calcAlignmentScore(in2, final) << endl;

  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud(convert(final, 255, 255, 0), "final");
  viewer.showCloud(convert(in2, 128, 128, 128), "target");
  while (!viewer.wasStopped()) ;

  return 0;
}
