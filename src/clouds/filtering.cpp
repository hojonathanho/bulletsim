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
using namespace pcl;
using boost::shared_ptr;
using namespace Eigen;

ColorCloudPtr maskCloud(const ColorCloudPtr in, const cv::Mat& mask) {
  assert(mask.elemSize() == 1);
  assert(mask.rows == in->height);
  assert(mask.cols == in->width);
  assert(in->isOrganized());

  shared_ptr< vector<int> > indicesPtr(new vector<int>());

  MatConstIterator_<bool> it = mask.begin<bool>(), it_end = mask.end<bool>();
  for (int i=0; it != it_end; ++it, ++i) {
    if (*it > 0) indicesPtr->push_back(i);
  }

  ColorCloudPtr out(new ColorCloud());
  pcl::ExtractIndices<pcl::PointXYZRGBA> ei;
  ei.setNegative(false);
  ei.setInputCloud(in);
  ei.setIndices(indicesPtr);
  ei.filter(*out);
  return out;
}


ColorCloudPtr maskCloud(const ColorCloudPtr in, const VectorXb& mask) {

  ColorCloudPtr out(new ColorCloud());
  int nOut = mask.sum();
  out->reserve(nOut);
  out->header=in->header;
  out->width = nOut;
  out->height = 1;
  out->is_dense = false;

  int i = 0;
  BOOST_FOREACH(const PointXYZRGBA& pt, in->points) {
    if (mask(i)) out->push_back(pt);
    ++i;
  }

  return out;
}

ColorCloudPtr downsampleCloud(const ColorCloudPtr in, float sz) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGBA>());
 pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
 vg.setInputCloud(in);
 vg.setLeafSize(sz,sz,sz);
 vg.filter(*out);
 return out;
}

ColorCloudPtr removeOutliers(const ColorCloudPtr in, float thresh, int k) {
  ColorCloudPtr out(new ColorCloud());
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
  sor.setInputCloud (in);
  sor.setMeanK (k);
  sor.setStddevMulThresh (thresh);
  sor.filter (*out);
  cout << "removeOutliers: removed " << (in->size() - out->size()) << " of " << in->size() << endl;
  return out;
}

void labelCloud(ColorCloudPtr in, const cv::Mat& labels) {
  ColorCloudPtr out(new ColorCloud());
  MatrixXi uv = xyz2uv(toEigenMatrix(in));
  for (int i=0; i < in->size(); i++)
    in->points[i]._unused = labels.at<uint8_t>(uv(i,0), uv(i,1));
}

