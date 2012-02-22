#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>


typedef pcl::PointCloud<pcl::PointXYZRGB> ColorCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr ConstColorCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorCloudPtr;

typedef Eigen::Matrix<bool,Eigen::Dynamic,1> VectorXb;
typedef Eigen::Matrix<uint8_t,Eigen::Dynamic,Eigen::Dynamic> MatrixXb;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPCD(const std::string& pcdfile);

Eigen::MatrixXi xyz2uv(const Eigen::MatrixXf& xyz);

Eigen::MatrixXf toEigenMatrix(ColorCloudPtr);
Eigen::MatrixXf getDepthImage(ColorCloudPtr);
pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(const std::vector< std::vector<float> >&);

MatrixXb toBGR(ColorCloudPtr);
cv::Mat toCVMat(Eigen::MatrixXf);


bool pointIsFinite(const pcl::PointXYZRGB& pt);
