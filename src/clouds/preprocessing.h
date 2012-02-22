#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Geometry>
#include "utils_pcl.h"


VectorXb getPointsOnTable(ColorCloudPtr cloudCam, const Eigen::Affine3f& camToWorld, const Eigen::MatrixXf& tableCam, float below = .01, float above=1);
