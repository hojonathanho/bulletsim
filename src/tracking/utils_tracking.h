#pragma once
#include <btBulletDynamicsCommon.h>
#include <Eigen/Dense>
#include "utils/config.h"
#include "clouds/pcl_typedefs.h"
#include "simulation/environment.h"
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <boost/thread.hpp>

typedef Eigen::Matrix<uint8_t,Eigen::Dynamic,Eigen::Dynamic> MatrixXu;

class CoordinateTransformer {
public:
  CoordinateTransformer();
  CoordinateTransformer(const btTransform& worldFromCam);
  btTransform worldFromCamUnscaled;
  Eigen::Affine3f worldFromCamEigen;
  btTransform camFromWorldUnscaled;
  Eigen::Affine3f camFromWorldEigen;
  inline btVector3 toWorldFromCam(const btVector3& camVec) { return METERS * (worldFromCamUnscaled * camVec); }
  inline btVector3 toCamFromWorld(const btVector3& worldVec) { return worldFromCamUnscaled.inverse() * ( worldVec / METERS); }
  std::vector<btVector3> toWorldFromCamN(const std::vector<btVector3>& camVecs);
  std::vector<btVector3> toCamFromWorldN(const std::vector<btVector3>& worldVecs);
  Eigen::MatrixXf toCamFromWorldMatrixXf(const Eigen::MatrixXf&);
  void reset(const btTransform &wfc);
};

class ImageTopicRecorder {
	cv::VideoWriter m_video_writer;
	ros::Subscriber m_subscriber;
	cv::Mat m_last_image;
	boost::posix_time::time_duration m_cycle_time;
	boost::posix_time::ptime m_last_time;
	std::string m_full_filename;
	void callback(sensor_msgs::ImageConstPtr image_msg);

public:
	ImageTopicRecorder(ros::NodeHandle& nh, std::string image_topic);
	ImageTopicRecorder(ros::NodeHandle& nh, std::string image_topic, std::string full_filename);
};

std::vector<btVector3> scaleVecs(const std::vector<btVector3>&, float);
ColorCloudPtr scaleCloud(ColorCloudPtr, float);

Eigen::Affine3f Scaling3f(float s);
Eigen::MatrixXf pairwiseSquareDist(const Eigen::MatrixXf& x_m3, const Eigen::MatrixXf& y_n3);
std::vector<int> argminAlongRows(const Eigen::MatrixXf& d_mn);

template <typename T, int S>
bool isFinite(const Eigen::Matrix<T, Eigen::Dynamic, S>& x) {
  for (int row=0; row < x.rows(); row++) for (int col=0; col<x.cols(); col++) if (!isfinite(x(row,col))) return false;
  return true;
}

std::vector<btVector3> toBulletVectors(ColorCloudPtr in);
