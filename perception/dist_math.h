#pragma once
#include <vector>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

MatrixXf pairwiseSquareDist(const MatrixXf& x_m3, const MatrixXf& y_n3);
vector<int> argminAlongRows(const MatrixXf& d_mn);
