#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "pcl_typedefs.h"
#include <LinearMath/btTransform.h>
#include <sensor_msgs/PointCloud2.h>

typedef Eigen::Matrix<bool,Eigen::Dynamic,1> VectorXb;
typedef Eigen::Matrix<uint8_t,Eigen::Dynamic,Eigen::Dynamic> MatrixXu;

ColorCloudPtr readPCD(const std::string& pcdfile);

Eigen::MatrixXi xyz2uv(const Eigen::MatrixXf& xyz);

Eigen::MatrixXf toEigenMatrix(ColorCloudPtr);
Eigen::MatrixXf toEigenMatrix(const cv::Mat&);
Eigen::MatrixXf getDepthImage(ColorCloudPtr);
pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(const std::vector< std::vector<float> >&);

MatrixXu toBGR(ColorCloudPtr);
cv::Mat toCVMat(Eigen::MatrixXf);
cv::Mat toCVMatImage(const ColorCloudPtr cloud);
cv::Mat toCVMatDepthImage(const ColorCloudPtr cloud);

bool pointIsFinite(const ColorPoint& pt);

inline ColorCloudPtr removeConst(ConstColorCloudPtr cloud) {
  return boost::const_pointer_cast< pcl::PointCloud<ColorPoint> >(cloud);
}
ColorCloudPtr transformPointCloud1(ColorCloudPtr in, Eigen::Affine3f transform);
ColorCloudPtr extractInds(ColorCloudPtr in, std::vector<int> inds);

bool saveTransform(const std::string& filename, const Eigen::Matrix4f& t);
bool loadTransform(const std::string& filename, Eigen::Matrix4f& t);
bool saveTransform(const std::string& filename, const Eigen::Affine3f& t);
bool loadTransform(const std::string& filename, Eigen::Affine3f& t);
inline ColorPoint toColorPoint(const Eigen::Vector3f& vec) {
    ColorPoint pt;
    pt.x = vec(0);
    pt.y = vec(1);
    pt.z = vec(2);
    return pt;
}
inline ColorPoint toColorPoint(const btVector3& vec) {
    ColorPoint pt;
    pt.x = vec.x();
    pt.y = vec.y();
    pt.z = vec.z();
    return pt;
}
inline Eigen::Vector3f toEigenVector(const ColorPoint& pt) {return Eigen::Vector3f(pt.x,pt.y,pt.z);}

ColorCloudPtr fromROSMsg1(const sensor_msgs::PointCloud2& msg);
