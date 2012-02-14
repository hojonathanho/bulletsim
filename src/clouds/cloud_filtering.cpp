#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "utils/my_assert.h"
#include "utils_pcl.h"
#include <iostream>

using namespace std;
using namespace cv;
using boost::shared_ptr;

ColorCloudPtr maskCloud(const ColorCloudPtr in, const cv::Mat& mask) {
  ENSURE(mask.elemSize() == 1);
  ENSURE(mask.rows == in->height);
  ENSURE(mask.cols == in->width);
  ENSURE(in->isOrganized());

  shared_ptr< vector<int> > indicesPtr(new vector<int>());

  MatConstIterator_<bool> it = mask.begin<bool>(), it_end = mask.end<bool>();
  for (int i=0; it != it_end; it++) {
    if (*it > 0) indicesPtr->push_back(i);
    i++;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::ExtractIndices<pcl::PointXYZRGB> ei;
  ei.setInputCloud(in);
  ei.setIndices(indicesPtr);
  ei.filter(*out);
  return out;
}

ColorCloudPtr downsampleCloud(const ColorCloudPtr in, float sz) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>());
 pcl::VoxelGrid<pcl::PointXYZRGB> vg;
 vg.setInputCloud(in);
 vg.setLeafSize(sz,sz,sz);
 vg.filter(*out);
 return out;
}

ColorCloudPtr removeOutliers(const ColorCloudPtr in, float thresh, int k) {
  ColorCloudPtr out(new ColorCloud());
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (in);
  sor.setMeanK (k);
  sor.setStddevMulThresh (thresh);
  sor.filter (*out);
  cout << "removeOutliers: removed " << (in->size() - out->size()) << " of " << in->size() << endl;
  return out;
}

