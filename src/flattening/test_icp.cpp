// Adapted from http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
// Takes point clouds A and B, aligns A to B, and displays 
// usage: test_icp A B C

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include "clouds/utils_pcl.h"

static double getFitnessScore(ColorCloudPtr input, ColorCloudPtr target, double max_range=std::numeric_limits<double>::max()) {
  double fitness_score = 0.0;

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  pcl::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGBA>);
  tree->setInputCloud(target);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < input->points.size (); ++i)
  {
    Eigen::Vector4f p1 = Eigen::Vector4f (input->points[i].x,
                                          input->points[i].y,
                                          input->points[i].z, 0);
    // Find its nearest neighbor in the target
    tree->nearestKSearch (input->points[i], 1, nn_indices, nn_dists);

    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] > max_range)
      continue;

    Eigen::Vector4f p2 = Eigen::Vector4f (target->points[nn_indices[0]].x,
                                          target->points[nn_indices[0]].y,
                                          target->points[nn_indices[0]].z, 0);
    // Calculate the fitness score
    fitness_score += fabs ((p1-p2).squaredNorm ());
    nr++;
  }

  if (nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max ());
}

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
  cout << "target -> aligned score: " << getFitnessScore(in2, final) << endl;

  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud(convert(final, 255, 255, 0), "final");
  viewer.showCloud(convert(in2, 128, 128, 128), "target");
  while (!viewer.wasStopped()) ;

  return 0;
}
