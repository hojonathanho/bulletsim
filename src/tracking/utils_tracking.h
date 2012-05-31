#pragma once
#include <btBulletDynamicsCommon.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include "clouds/utils_pcl.h"

void toggle(bool* b);

class CoordinateTransformer {
public:
  CoordinateTransformer(const btTransform& worldFromCam);
  btTransform worldFromCamUnscaled;
  Eigen::Affine3f worldFromCamEigen;
  btTransform camFromWorldUnscaled;
  Eigen::Affine3f camFromWorldEigen;
  inline btVector3 toWorldFromCam(const btVector3& camVec);
  inline btVector3 toCamFromWorld(const btVector3& camVec);
  std::vector<btVector3> toWorldFromCamN(const std::vector<btVector3>& camVecs);
  std::vector<btVector3> toCamFromWorldN(const std::vector<btVector3>& worldVecs);
  Eigen::MatrixXf toCamFromWorldMatrixXf(const Eigen::MatrixXf&);
  void reset(const btTransform &wfc);
};

std::vector<btVector3> scaleVecs(const std::vector<btVector3>&, float);
ColorCloudPtr scaleCloud(ColorCloudPtr, float);

Eigen::Affine3f Scaling3f(float s);
Eigen::MatrixXf pairwiseSquareDist(const Eigen::MatrixXf& x_m3, const Eigen::MatrixXf& y_n3);
std::vector<int> argminAlongRows(const Eigen::MatrixXf& d_mn);
bool isFinite(const Eigen::MatrixXf& x);

std::vector<btVector3> toBulletVectors(ColorCloudPtr in);


btTransform waitForAndGetTransform(const tf::TransformListener& listener, std::string target_frame, std::string source_frame);
