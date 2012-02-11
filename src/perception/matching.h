#pragma once
#include <vector>
#include <Eigen/Dense>
// #include "sparse_array.h"
using namespace std;

Eigen::VectorXi matchHardMaximal(const Eigen::MatrixXf& costs); 
// match every member of smaller set to some member of the larger set

Eigen::VectorXi matchHardOneWay(const Eigen::MatrixXf& costs);
// match every member of first set (rows) to second set (columns)

Eigen::VectorXi matchHard(const Eigen::MatrixXf& costs, float missPenalty);
// match some members of first set (rows) to second set (columns), where penalty for skipping a row is missPenalty

// SparseArray matchSoft(const Eigen::MatrixXf& costs,float missingPenaltyA=500, float missingPenaltyB=500);
// SparseArray matchSoft(const SparseArray& costs, float missingPenaltyA=500, float missingPenaltyB=500);