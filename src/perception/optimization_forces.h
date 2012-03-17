#pragma once

#include <vector>
#include <btBulletDynamicsCommon.h>
#include "sparse_array.h"
#include "simulation/plotting.h"
#include "plotting_perception.h"
using std::vector;


SparseArray calcCorrOpt(const vector<btVector3>& estPts, const vector<btVector3>& obsPts, const vector<float>& pVis);
SparseArray calcCorrNN(const vector<btVector3>& estPts, const vector<btVector3>& obsPts, const vector<float>& pVis);

vector<btVector3> calcImpulsesSimple(const vector<btVector3>& estPts, const vector<btVector3>& obsPts, const SparseArray& corr, float f);
vector<btVector3> calcImpulsesDamped(const vector<btVector3>& estPos, const vector<btVector3>& estVel, const vector<btVector3>& obsPts, const SparseArray& corr, vector<float> masses, float kp, float kd);

// todo: normalization factor in likelihood
Eigen::MatrixXf calcCorrProb(const Eigen::MatrixXf& estPts, const Eigen::VectorXf& sigs, const Eigen::MatrixXf& obsPts, const Eigen::VectorXf& pVis, float pBandOutlier, float& loglik);
Eigen::MatrixXf calcCorrProb(const Eigen::MatrixXf& estPts, const Eigen::VectorXf& sigs, const Eigen::MatrixXf& obsPts, const Eigen::VectorXf& pVis, float pBandOutlier);
Eigen::MatrixXf calcCorrProb(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::VectorXf& pVis, float stdev, float pBandOutlier);
Eigen::VectorXf calcSigs(const SparseArray& corr, const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, float priorDist, float priorCount);

class CorrPlots {
public:
  PlotLines::Ptr m_lines;
  PointCloudPlot::Ptr m_sources;
  PointCloudPlot::Ptr m_targs;

  CorrPlots();
  void update(const vector<btVector3>&, const vector<btVector3>&, const SparseArray&);
};
