#pragma once
#include <Eigen/Dense>
#include "sparse_utils.h"
#include "clouds/pcl_typedefs.h"

Eigen::MatrixXf calculateStdevNaive(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::MatrixXf& pZgivenC, const Eigen::VectorXf& dPrior, const float& nuPrior);
Eigen::MatrixXf calculateStdev(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::MatrixXf& pZgivenC, const Eigen::VectorXf& dPrior, const float& nuPrior);
Eigen::MatrixXf calculateResponsibilitiesNaive(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::MatrixXf& stdev, const Eigen::VectorXf& pVis, const Eigen::VectorXf& outlierDist, const Eigen::VectorXf& outlierStdev);
Eigen::MatrixXf calculateResponsibilities(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::MatrixXf& stdev, const Eigen::VectorXf& pVis, const Eigen::VectorXf& outlierDist, const Eigen::VectorXf& outlierStdev);

Eigen::MatrixXf calcSigs(const SparseMatrixf& corr, const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, Eigen::VectorXf priorDist, float priorCount);

Eigen::MatrixXf calcSigsEigen(const SparseMatrixf& corr, const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, Eigen::VectorXf priorDist, float priorCount);

void estimateCorrespondenceFull(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& stdev, const Eigen::VectorXf& pVis,
  const Eigen::MatrixXf& obsPts, const Eigen::VectorXf& outlierDist, Eigen::MatrixXf& pZgivenB, SparseMatrixf& corr);

void estimateCorrespondence(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& stdev, const Eigen::VectorXf& pVis,
	const Eigen::MatrixXf& obsPts, const Eigen::VectorXf& outlierDist, Eigen::MatrixXf& pZgivenB, SparseMatrixf& corr);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////// The functions below are experimental only as they apply for special cases //////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void estimateCorrespondenceSame(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& sigma, const Eigen::VectorXf& pVis,
  const Eigen::MatrixXf& obsPts, float pBandOutlier, SparseMatrixf& corr);

void estimateCorrespondenceCloud (ColorCloudPtr cloud, const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& sigma, const Eigen::VectorXf& pVis,
	  const Eigen::MatrixXf& obsPts, float pBandOutlier, SparseMatrixf& corr, int K);

void estimateCorrespondence(const Eigen::MatrixXf& estPts, const Eigen::VectorXf& stdev, const Eigen::VectorXf& vis, 
  const Eigen::MatrixXf& m_obsPts, float pBandOutlier, SparseMatrixf& corr);
