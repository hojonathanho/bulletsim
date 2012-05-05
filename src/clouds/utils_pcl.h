#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>


typedef pcl::PointCloud<pcl::PointXYZRGBA> ColorCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr ConstColorCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ColorCloudPtr;

typedef Eigen::Matrix<bool,Eigen::Dynamic,1> VectorXb;
typedef Eigen::Matrix<uint8_t,Eigen::Dynamic,Eigen::Dynamic> MatrixXu;

ColorCloudPtr readPCD(const std::string& pcdfile);

Eigen::MatrixXi xyz2uv(const Eigen::MatrixXf& xyz);

Eigen::MatrixXf toEigenMatrix(ColorCloudPtr);
Eigen::MatrixXf getDepthImage(ColorCloudPtr);
pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(const std::vector< std::vector<float> >&);

MatrixXu toBGR(ColorCloudPtr);
cv::Mat toCVMat(Eigen::MatrixXf);


bool pointIsFinite(const pcl::PointXYZRGBA& pt);

inline ColorCloudPtr removeConst(ConstColorCloudPtr cloud) {
  return boost::const_pointer_cast< pcl::PointCloud<pcl::PointXYZRGBA> >(cloud);
}
ColorCloudPtr transformPointCloud1(ColorCloudPtr in, Eigen::Affine3f transform);
ColorCloudPtr extractInds(ColorCloudPtr in, std::vector<int> inds);
