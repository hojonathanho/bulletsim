#include "utils_pcl.h"
#include <pcl/io/pcd_io.h>
#include "utils/my_exceptions.h"

using namespace Eigen;
using namespace pcl;
using namespace std;

pcl::PointCloud<ColorPoint>::Ptr readPCD(const std::string& pcdfile) {
  pcl::PointCloud<ColorPoint>::Ptr cloud(new pcl::PointCloud<ColorPoint>);
  if (pcl::io::loadPCDFile<ColorPoint> (pcdfile, *cloud) == -1) {
    throw FileOpenError(pcdfile);
    }
  return cloud;
}


static const float cx = 320-.5;
static const float cy = 240-.5;
static const float f = 525;

Eigen::MatrixXi xyz2uv(const Eigen::MatrixXf& xyz) {
 // http://www.pcl-users.org/Using-Kinect-with-PCL-How-to-project-a-3D-point-x-y-z-to-the-depth-rgb-image-and-how-to-unproject-a--td3164499.html
  VectorXf x = xyz.col(0);
  VectorXf y = xyz.col(1);
  VectorXf z = xyz.col(2);

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

MatrixXu toBGR(ColorCloudPtr cloud) {
  MatrixXf bgrFloats = cloud->getMatrixXfMap(1,8,4);
  MatrixXu bgrBytes4 = Map<MatrixXu>(reinterpret_cast<uint8_t*>(bgrFloats.data()), bgrFloats.rows(),4);
  MatrixXu bgrBytes3 = bgrBytes4.block(0,0,bgrBytes4.rows(),3);
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

MatrixXf toEigenMatrix(const cv::Mat& in) {
	if (in.type() != CV_32FC1) throw runtime_error("input matrix has the wrong type");
	return Map<MatrixXf>((float*)in.data, in.rows, in.cols);
}

cv::Mat toCVMatImage(const ColorCloudPtr cloud) {
	MatrixXu bgr = toBGR(cloud);
	cv::Mat image(cloud->height, cloud->width, CV_8UC3, bgr.data());
	return image;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(const std::vector< std::vector<float> >& in) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
  BOOST_FOREACH(vector<float> v, in) out->push_back(PointXYZ(v[0],v[1],v[2]));
  out->width=in.size();
  out->height=1;
  return out;
}

bool pointIsFinite(const ColorPoint& pt) {
  return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);
}

ColorCloudPtr transformPointCloud1(ColorCloudPtr in, Eigen::Affine3f transform) {
  ColorCloudPtr out(new ColorCloud(*in));
  BOOST_FOREACH(ColorPoint& p, out->points) {
    p.getVector3fMap() = transform * p.getVector3fMap();
  }
  return out;
}

ColorCloudPtr extractInds(ColorCloudPtr in, std::vector<int> inds) {
  ColorCloudPtr out(new ColorCloud());
  out->reserve(inds.size());
  out->width = inds.size();
  out->height = 1;
  out->is_dense = false;

  BOOST_FOREACH(int i, inds) {
    out->push_back(in->points[i]);
  }
  return out;
}
