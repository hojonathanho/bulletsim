#pragma once
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "utils_pcl.h"

Eigen::MatrixXf quadFeats(const Eigen::MatrixXf& X);

Eigen::MatrixXf correctPoints(const Eigen::MatrixXf& X, const Eigen::MatrixXf& coefs);

ColorCloudPtr correctCloudXYZRGBA(ColorCloudPtr in, const Eigen::MatrixXf& coefs);
