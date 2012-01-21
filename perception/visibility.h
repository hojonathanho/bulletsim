#include <Eigen/Dense>
#include <vector>
#include "rope.h"

std::vector<float> calcVisibility(const Eigen::MatrixXf& pts, const Eigen::MatrixXf& depth, const cv::Mat& ropeMask);
