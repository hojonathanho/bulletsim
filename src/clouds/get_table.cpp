#include "get_table.h"
#include <boost/foreach.hpp>
#include "geom.h"
#include <iostream>
#include <vector>
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


void getTable(PointCloud<PointXYZRGB>::Ptr cloud, vector<Vector3f>& corners, Vector3f& normal) {
  // downsample -> cloud_down
  // get nearby points -> cloud_near
  // get plane -> cloud_table
  // project to 2d, get rectangle

  PointCloud<PointXYZRGB>::Ptr cloud_down(new PointCloud<PointXYZRGB>);

  VoxelGrid<PointXYZRGB> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(.01,.01,.01);
  vg.filter(*cloud_down);

  PointCloud<PointXYZRGB>::Ptr cloud_near(new PointCloud<PointXYZRGB>);
  PassThrough<PointXYZRGB> extract_near;
  extract_near.setInputCloud(cloud_down);
  extract_near.setFilterFieldName("z");
  extract_near.setFilterLimits(0,2);
  extract_near.setFilterLimitsNegative(false);
  extract_near.filter(*cloud_near);


  //PointCloud<PointXYZRGB>::Ptr cloud_plane(new PointCloud<PointXYZRGB>);
  pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  SACSegmentation<PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(SACMODEL_PLANE);
  seg.setMethodType(SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(.0025);
  seg.setInputCloud(cloud_near);
  seg.segment(*inliers,*coeffs);
  // ExtractIndices<PointXYZRGB> extract_plane;
  // extract_plane.setInputCloud(cloud_near);
  // extract_plane.setIndices(inliers);
  // extract_plane.setNegative(false);
  // extract_plane.filter(*cloud_plane);

  PointCloud<PointXYZRGB>::Ptr cloud_plane2(new PointCloud<PointXYZRGB>);
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_near);
  proj.setModelCoefficients (coeffs);
  proj.filter (*cloud_plane2);

  PointCloud<PointXYZRGB>::Ptr cloud_cluster(new PointCloud<PointXYZRGB>);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (500);
  ec.setMaxClusterSize (250000);
  ec.setSearchMethod (tree);
  ec.setInputCloud(cloud_plane2);
  ec.extract (cluster_indices);
  std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
  cout << "warning in get_table--wtf?" << endl;
  it++;

  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
    cloud_cluster->push_back(cloud_plane2->points[*pit]);
  cloud_cluster->width = cloud_cluster->points.size ();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;


  vector<Vector3f> tablePts;
  Vector4f abcd(coeffs->values[0],coeffs->values[1],coeffs->values[2],coeffs->values[3]);
  vector<Vector3f> tableVerts;
  BOOST_FOREACH(PointXYZRGB pt, *cloud_cluster) tablePts.push_back(Vector3f(pt.x,pt.y,pt.z));


  minEncRect(tablePts,abcd,corners);
  cout << "corners.size" << corners.size() << endl;
  normal = Vector3f(abcd[0],abcd[1],abcd[2]);
  normal.normalize();


}
