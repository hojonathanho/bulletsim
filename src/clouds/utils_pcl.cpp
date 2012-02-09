#include "utils_pcl.h"
#include <pcl/io/pcd_io.h>
#include "my_exceptions.h"

using namespace Eigen;
using namespace pcl;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPCD(const std::string& pcdfile) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdfile, *cloud) == -1) {
    throw FileOpenError(pcdfile);
    }
  return cloud;
}

Eigen::MatrixXi xyz2uv(const Eigen::MatrixXf& xyz) {
 // http://www.pcl-users.org/Using-Kinect-with-PCL-How-to-project-a-3D-point-x-y-z-to-the-depth-rgb-image-and-how-to-unproject-a--td3164499.html
  VectorXf x = xyz.col(0);
  VectorXf y = xyz.col(1);
  VectorXf z = xyz.col(2);

  float cx = 320-.5;
  float cy = 240-.5;
  float f = 525;
  VectorXf v = f*(x.array() / z.array()) + cx;
  VectorXf u = f*(y.array() / z.array()) + cy;
  MatrixXi uv(u.rows(),2);
  uv.col(0) = u.cast<int>();
  uv.col(1) = v.cast<int>();
  return uv;
}

Eigen::MatrixXf toEigenMatrix(ColorCloudPtr cloud) {
  return cloud->getMatrixXfMap(3,8,0);
}


MatrixXb toBGR(ColorCloudPtr cloud) {
  MatrixXf bgrFloats = cloud->getMatrixXfMap(1,8,4);
  MatrixXb bgrBytes4 = Map<MatrixXb>(reinterpret_cast<uint8_t*>(bgrFloats.data()), bgrFloats.rows(),4);
  MatrixXb bgrBytes3 = bgrBytes4.block(0,0,bgrBytes4.rows(),3);
  return bgrBytes3;
}

MatrixXf getDepthImage(ColorCloudPtr cloud) {
  MatrixXf xyz = toEigenMatrix(cloud);
  MatrixXf norms = xyz.rowwise().norm();
  norms.resize(cloud->height, cloud->width);
  return norms;
}

cv::Mat toCVMat(Eigen::MatrixXf in) {
  return cv::Mat(in.rows(), in.cols(), CV_32FC1, in.data());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(const std::vector< std::vector<float> >& in) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
  BOOST_FOREACH(vector<float> v, in) out->push_back(PointXYZ(v[0],v[1],v[2]));
  out->width=in.size();
  out->height=1;
  return out;
}
