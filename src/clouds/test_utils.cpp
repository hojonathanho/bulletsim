#include <cmath>
#include "utils_pcl.h"
#include "utils/testing.h"
#include "utils/my_assert.h"
using namespace Eigen;
using namespace pcl;

void test_toEigenMatrix() {
ColorCloudPtr cloud = readPCD("/home/joschu/cpp/clouds/test.pcd");
 MatrixXf xyz = toEigenMatrix(cloud);
 ENSURE(xyz.rows()==cloud->size());
 ENSURE(xyz.cols()==3);
 for (int i=0; i<cloud->size(); i++) {
    PointXYZRGBA& pt = cloud->at(i);
    ENSURE(pt.x == xyz(i,0));
    ENSURE(pt.y == xyz(i,1));
    ENSURE(pt.z == xyz(i,2));

 }
}

void test_toBGR() {
  ColorCloudPtr cloud = readPCD("/home/joschu/cpp/clouds/test.pcd");
  MatrixXb bgr = toBGR(cloud);
  ENSURE(bgr.rows() == cloud->size());
  ENSURE(bgr.cols() == 3);
  for (int i=0; i<cloud->size(); i++) {
    PointXYZRGBA& pt = cloud->at(i);
    ENSURE(pt.b == bgr(i,0));
    ENSURE(pt.g == bgr(i,1));
    ENSURE(pt.r == bgr(i,2));
  }
}

void test_getDepthImage() {
  ColorCloudPtr cloud = readPCD("/home/joschu/cpp/clouds/test.pcd");
  MatrixXf depths = getDepthImage(cloud);
  depths.resize(depths.rows()*depths.cols(),1);
  for (int i=0; i<cloud->size(); i++){
      PointXYZRGBA& pt = cloud->at(i);
      float d = sqrtf(pow(pt.x,2)+pow(pt.y,2)+pow(pt.z,2));
      ENSURE(d == depths(i));
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
