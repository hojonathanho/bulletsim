#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Geometry>
#include "utils_pcl.h"
#include "clouds/comm_pcl.h"


VectorXb getPointsOnTable(ColorCloudPtr cloudCam, const Eigen::Affine3f& camToWorld, const Eigen::MatrixXf& tableCam, float below = .01, float above=1);


class TowelPreprocessor {
public:
  
  Eigen::MatrixXf m_coeffs;
  Eigen::VectorXf m_intercepts;
  Eigen::MatrixXf m_cornersWorld;
  Eigen::Affine3f m_camToWorld;
  
  TowelPreprocessor();
  ColorCloudPtr extractTowelPoints(ColorCloudPtr cloud);

};

class RopePreprocessor {
public:
  
  Eigen::MatrixXf m_coeffs;
  Eigen::VectorXf m_intercepts;
  Eigen::MatrixXf m_cornersWorld;
  Eigen::Affine3f m_camToWorld;
  
  RopePreprocessor();
  ColorCloudPtr extractRopePoints(ColorCloudPtr cloud);

};


class RopePreprocessor2 {
public:
  
  Eigen::MatrixXf m_cornersWorld;
  Eigen::Affine3f m_camToWorld;
  
  RopePreprocessor2();
  ColorCloudPtr extractRopePoints(ColorCloudPtr cloud);

};

