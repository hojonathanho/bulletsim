#pragma once
#include <Eigen/Dense>
#include "sparse_utils.h"

void estimateCorrespondence(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& sigma, const Eigen::VectorXf& pVis,
  const Eigen::MatrixXf& obsPts, float pBandOutlier, SparseMatrixf& corr);

void estimateCorrespondence(const Eigen::MatrixXf& estPts, const Eigen::VectorXf& stdev, const Eigen::VectorXf& vis, 
  const Eigen::MatrixXf& m_obsPts, float pBandOutlier, SparseMatrixf& corr);
