#include "get_table.h"
#include <boost/foreach.hpp>
#include "geom.h"
#include <iostream>
#include <vector>
#include <sstream>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/project_inliers.h>

using namespace pcl;
using namespace std;
using namespace Eigen;

VectorXf toVectorXf1(const vector<float>& in) {
  VectorXf out(in.size());
  for (int i=0; i < in.size(); i++) out(i) = in[i];
  return out;
}

void getTable(PointCloud<ColorPoint>::Ptr cloud, vector<Vector3f>& corners, Vector3f& normal, int skip) {
  // downsample -> cloud_down
  // get nearby points -> cloud_near
  // get plane -> cloud_table
  // project to 2d, get rectangle

  PointCloud<ColorPoint>::Ptr cloud_down(new PointCloud<ColorPoint>);

  VoxelGrid<ColorPoint> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(.01,.01,.01);
  vg.filter(*cloud_down);

  PointCloud<ColorPoint>::Ptr cloud_near(new PointCloud<ColorPoint>);
  PassThrough<ColorPoint> extract_near;
  extract_near.setInputCloud(cloud_down);
  extract_near.setFilterFieldName("x");
  extract_near.setFilterLimits(-.25,.25);
  extract_near.setFilterLimitsNegative(false);
  extract_near.filter(*cloud_near);


  pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  SACSegmentation<ColorPoint> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(SACMODEL_PLANE);
  seg.setMethodType(SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(.0025);
  seg.setInputCloud(cloud_near);
  seg.segment(*inliers,*coeffs); 

  PointCloud<ColorPoint>::Ptr cloud_plane(new PointCloud<ColorPoint>);

  Vector4f coeffsEigen = toVectorXf1(coeffs->values);
  BOOST_FOREACH(const ColorPoint& pt, cloud_down->points) {
    Vector4f xyz1 = pt.getVector4fMap();
    xyz1(3) = 1;
    if (fabs(xyz1.dot(coeffsEigen)) < .01) cloud_plane->push_back(pt);
  }

  cloud_plane->width = cloud_plane->size();
  cloud_plane->height = 1;

  // pcl::ProjectInliers<ColorPoint> proj;
  // proj.setModelType(SACMODEL_PLANE);
  // proj.setModelCoefficients(coeffs);
  // proj.setInputCloud (cloud_down);
  // proj.setIndices(inliers1);
  // proj.setModelCoefficients (coeffs);
  // proj.filter (*cloud_plane);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<ColorPoint> ec;
  pcl::search::KdTree<ColorPoint>::Ptr tree (new pcl::search::KdTree<ColorPoint>);
  ec.setClusterTolerance (0.02);
  ec.setMinClusterSize (500);
  ec.setMaxClusterSize (250000);
  ec.setSearchMethod (tree);
  ec.setInputCloud(cloud_plane);
  ec.extract (cluster_indices);



  PointCloud<ColorPoint>::Ptr cloud_cluster(new PointCloud<ColorPoint>);

  BOOST_FOREACH(int pind, cluster_indices[skip].indices) {
    cloud_cluster->push_back(cloud_plane->points[pind]);
  }

  cloud_cluster->width = cloud_cluster->points.size ();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;


  vector<Vector3f> tablePts;
  Vector4f abcd(coeffs->values[0],coeffs->values[1],coeffs->values[2],coeffs->values[3]);
  vector<Vector3f> tableVerts;
  BOOST_FOREACH(ColorPoint pt, *cloud_cluster) tablePts.push_back(Vector3f(pt.x,pt.y,pt.z));


  minEncRect(tablePts,abcd,corners);
  cout << "corners.size" << corners.size() << endl;
  normal = Vector3f(abcd[0],abcd[1],abcd[2]);
  normal.normalize();


}
