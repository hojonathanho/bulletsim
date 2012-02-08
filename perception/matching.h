#pragma once
#include <vector>
#include <Eigen/Dense>
#include "sparse_array.h"
using namespace std;

vector<int> matchHardMaximal(const Eigen::MatrixXf& costs); 
// match every member of smaller set to some member of the larger set

vector<int> matchHardOneWay(const Eigen::MatrixXf& costs);

SparseArray matchSoft(const Eigen::MatrixXf& costs,float missingPenaltyA=500, float missingPenaltyB=500);
SparseArray matchSoft(const SparseArray& costs, float missingPenaltyA=500, float missingPenaltyB=500);
