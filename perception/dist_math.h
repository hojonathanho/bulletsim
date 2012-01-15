#pragma once
#include <vector>
#include <Eigen/Dense>
using namespace std;

Eigen::MatrixXf pairwiseSquareDist(const Eigen::MatrixXf& x_m3, const Eigen::MatrixXf& y_n3);
vector<int> argminAlongRows(const Eigen::MatrixXf& d_mn);
