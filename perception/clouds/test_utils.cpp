#include "utils_pcl.h"
#include "testing.h"
#include <cmath>
#include "my_assert.h"
using namespace Eigen;
using namespace pcl;

void test_toEigenMatrix() {
ColorCloudPtr cloud = readPCD("/home/joschu/cpp/clouds/test.pcd");
 MatrixXf xyz = toEigenMatrix(cloud);
 ASSERT(xyz.rows()==cloud->size());
 ASSERT(xyz.cols()==3);
 for (int i=0; i<cloud->size(); i++) {
    PointXYZRGB& pt = cloud->at(i);
    ASSERT(pt.x == xyz(i,0));
    ASSERT(pt.y == xyz(i,1));
    ASSERT(pt.z == xyz(i,2));

 }
}

void test_toBGR() {
  ColorCloudPtr cloud = readPCD("/home/joschu/cpp/clouds/test.pcd");
  MatrixXb bgr = toBGR(cloud);
  ASSERT(bgr.rows() == cloud->size());
  ASSERT(bgr.cols() == 3);
  for (int i=0; i<cloud->size(); i++) {
    PointXYZRGB& pt = cloud->at(i);
    ASSERT(pt.b == bgr(i,0));
    ASSERT(pt.g == bgr(i,1));
    ASSERT(pt.r == bgr(i,2));
  }
}

void test_getDepthImage() {
  ColorCloudPtr cloud = readPCD("/home/joschu/cpp/clouds/test.pcd");
  MatrixXf depths = getDepthImage(cloud);
  depths.resize(depths.rows()*depths.cols(),1);
  for (int i=0; i<cloud->size(); i++){
      PointXYZRGB& pt = cloud->at(i);
      float d = sqrtf(pow(pt.x,2)+pow(pt.y,2)+pow(pt.z,2));
      ASSERT(d == depths(i));
  }
}

void test_xyz2uv() {
  // todo
  // for now, just here to avoid linker errors
  MatrixXf x;
  xyz2uv(x);
}

int main() {
  TEST_FUNC(test_toEigenMatrix);
  TEST_FUNC(test_toBGR);
  TEST_FUNC(test_getDepthImage);
}
