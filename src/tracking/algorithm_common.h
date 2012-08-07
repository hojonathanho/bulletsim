#pragma once
#include <Eigen/Dense>
#include "sparse_utils.h"
#include "clouds/pcl_typedefs.h"
#include "tracking_defs.h"

class CloudFeatureExtractor {
public:
  std::vector<FeatureType> m_featureTypes;
  int m_featureDim;
  CloudFeatureExtractor();
  CloudFeatureExtractor(const std::vector<FeatureType>&);
  void setFeatureTypes(const std::vector<FeatureType>&);
  Eigen::MatrixXf extractFeatures(ColorCloudPtr in);
};

Eigen::MatrixXf calcSigs(const SparseMatrixf& corr, const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, Eigen::VectorXf priorDist, float priorCount);

Eigen::MatrixXf calcSigsEigen(const SparseMatrixf& corr, const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, Eigen::VectorXf priorDist, float priorCount);

void estimateCorrespondence(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& sigma, const Eigen::VectorXf& pVis,
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
