#pragma once
#include <vector>
#include <Eigen/Dense>
using namespace std;

class btVector3;

Eigen::MatrixXf pairwiseSquareDist(const Eigen::MatrixX3f& x_m3, const Eigen::MatrixX3f& y_n3);
vector<int> argminAlongRows(const Eigen::MatrixXf& d_mn);
vector<int> getNNInds(vector<btVector3> from, vector<btVector3> to);
