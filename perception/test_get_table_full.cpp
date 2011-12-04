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
#include <pcl/visualization/cloud_viewer.h>
#include <boost/foreach.hpp>
#include "geom.h"

using namespace pcl;
using namespace std;
using namespace Eigen;

void getTable(PointCloud<PointXYZRGB>::Ptr& cloud, vector<Vector3f>& corners, Vector3f& normal) {
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


  PointCloud<PointXYZRGB>::Ptr cloud_table(new PointCloud<PointXYZRGB>);
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
  ExtractIndices<PointXYZRGB> extract_table;
  extract_table.setInputCloud(cloud_near);
  extract_table.setIndices(inliers);
  extract_table.setNegative(false);
  extract_table.filter(*cloud_table);

  vector<Vector3f> tablePts;
  Vector4f abcd(coeffs->values[0],coeffs->values[1],coeffs->values[2],coeffs->values[3]);
  vector<Vector3f> tableVerts;
  BOOST_FOREACH(PointXYZRGB pt, *cloud_table) tablePts.push_back(Vector3f(pt.x,pt.y,pt.z));


  minEncRect(tablePts,abcd,corners);
  cout << "corners.size" << corners.size() << endl;
  normal = Vector3f(abcd[0],abcd[1],abcd[2]);
  normal.normalize();


}

int main() {
  PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
const string pcdfile = "/home/joschu/Data/rope_ends/blue_ends.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdfile, *cloud) == -1) {
    PCL_ERROR(("couldn't read file " + pcdfile + "\n").c_str());
    return -1;
  }
  vector<Vector3f> corners; 
  Vector3f normal;
  getTable(cloud,corners,normal);

  cout << "corners:" << endl;
  BOOST_FOREACH(Vector3f c, corners) cout << c;
  cout << endl;


  PointCloud<PointXYZ>::Ptr rectCloud(new PointCloud<PointXYZ>);
  BOOST_FOREACH(Vector3f w, corners) rectCloud->push_back(PointXYZ(w[0],w[1],w[2]));
  pcl::visualization::PCLVisualizer viewer ("Simple Cloud Viewer");
  viewer.addPointCloud (cloud);
  viewer.addPolygon<PointXYZ>(rectCloud,255,0,0);
  viewer.spin();

}
