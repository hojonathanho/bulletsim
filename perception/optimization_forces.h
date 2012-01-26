#pragma once

#include <vector>
#include <btBulletDynamicsCommon.h>
#include "sparse_array.h"
#include "plotting.h"
using std::vector;


SparseArray normalizeRows(const SparseArray&);
float sum(const SparseVector&);
float sum(const SparseArray&);

SparseArray calcCorrOpt(const vector<btVector3>& estPts, const vector<btVector3>& obsPts, const vector<float>& pVis);
SparseArray calcCorrNN(const vector<btVector3>& estPts, const vector<btVector3>& obsPts, const vector<float>& pVis);

vector<btVector3> calcImpulsesSimple(const vector<btVector3>& estPts, const vector<btVector3>& obsPts, const SparseArray& corr, float f);
vector<btVector3> calcImpulsesDamped(const vector<btVector3>& estPts, const vector<btVector3>& obsPts, const SparseArray& corr, float kp, float kd);
// todo: normalization factor in likelihood
Eigen::MatrixXf calcCorrProb(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::VectorXf& pVis, float stdev, float pBandOutlier);

Eigen::MatrixXf calcCorrProb(const Eigen::MatrixXf& estPts, const Eigen::MatrixXf& obsPts, const Eigen::VectorXf& pVis, float stdev, float outlierDensity);

class CorrPlots {
public:
  PlotLines::Ptr m_lines;
  CorrPlots();
  void update(const vector<btVector3>&, const vector<btVector3>&, const SparseArray&);
};
