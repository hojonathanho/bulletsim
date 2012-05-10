#include "clouds/plane_finding.h"
#include <boost/foreach.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

typedef ColorPoint PointT;
using namespace std;
using namespace Eigen;

vector<float> getPlaneCoeffsRansac(ColorCloudPtr cloud) {

  pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::SACSegmentation<ColorPoint> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(.0025);
  seg.setInputCloud(cloud);
  seg.segment(*inliers,*coeffs); 

  return coeffs->values;
}

vector<int> getPlaneInliers(ColorCloudPtr cloud, vector<float> coeffs) {
  vector<int> inds;
  Vector4f coeffsEigen = Vector4f::Map(coeffs.data());
  for (int i=0; i < cloud->size(); i++) {
    Vector4f xyz1 = cloud->points[i].getVector4fMap();
    xyz1(3) = 1;
    if (fabs(xyz1.dot(coeffsEigen)) < .01) inds.push_back(i);
  }
  return inds;  
}
