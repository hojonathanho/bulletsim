#include "optimization_forces.h"
#include "dist_math.h"
#include "utils_perception.h"
#include <Eigen/Dense>
#include <boost/foreach.hpp>
#include "matching.h"
using namespace std;
using namespace Eigen;

// todo: some weighting scheme
SparseArray calcCorrNN(const vector<btVector3>& estPts, const vector<btVector3>& obsPts, const vector<float>& pVis) {
  int nEst = estPts.size();
  int nObs = obsPts.size();
  float r = (float)nObs/nEst;
  SparseArray out(nEst);
  MatrixXf distsEstObs = pairwiseSquareDist(toEigenMatrix(estPts), toEigenMatrix(obsPts));
  vector<int> estToObs = argminAlongRows(distsEstObs);
  vector<int> obsToEst = argminAlongRows(distsEstObs.transpose());
  for (int iEst=0; iEst<nEst; iEst++) 
    out[iEst].push_back(IndVal(estToObs[iEst],.5*pVis[iEst]));
  for (int iObs=0; iObs<nObs; iObs++) out[obsToEst[iObs]].push_back(IndVal(iObs,.5/r));
  return out;
}

// todo: normalization factor in likelihood
MatrixXf calcCorrProb(const MatrixXf& estPts, const MatrixXf& obsPts, const VectorXf& pVis, float stdev, float pBandOutlier) {
  MatrixXf sqdists = pairwiseSquareDist(estPts, obsPts);
  MatrixXf pBgivenZ_unnormed = (-sqdists/(2*stdev)).array().exp();
  MatrixXf pBandZ_unnormed = pVis.asDiagonal()*pBgivenZ_unnormed;
  VectorXf pB_unnormed = pBandZ_unnormed.colwise().sum();
  VectorXf pBorOutlier_unnormed = (pB_unnormed.array() + pBandOutlier).inverse();
  MatrixXf pZgivenB = pBandZ_unnormed * pBorOutlier_unnormed.asDiagonal();
  //cout << pZgivenB.row(0);
  cout << stdev << endl;
  return pZgivenB;
}


SparseArray calcCorrOpt(const vector<btVector3>& estPts, const vector<btVector3>& obsPts, const vector<float>& pVis) {
  // todo: use pvis
  MatrixXf costs = pairwiseSquareDist(toEigenMatrix(estPts), toEigenMatrix(obsPts));
  SparseArray corr = matchSoft(costs,1,0);
  return corr;
}

vector<btVector3> calcImpulsesSimple(const vector<btVector3>& estPts, const vector<btVector3>& obsPts, const SparseArray& corr, float f) {
  int nEst = estPts.size();
  vector<btVector3> out(nEst, btVector3(0,0,0));
  for (int iEst=0; iEst < nEst; iEst++)
    BOOST_FOREACH(const IndVal& iv, corr[iEst]) 
      out[iEst] += f * iv.val * (obsPts[iv.ind] - estPts[iEst]);
  return out;
}

vector<btVector3> calcImpulsesDamped(const vector<btVector3>& estPos, const vector<btVector3>& estVel, const vector<btVector3>& obsPts, const SparseArray& corr, vector<float> masses, float kp, float kd) {
  SparseArray ncorr = normalizeRows(corr);
  int nEst = estPos.size();
  int nObs = obsPts.size();
  vector<btVector3> out(nEst);
  for (int iEst=0; iEst < nEst; iEst++) {
    btVector3 dv = -kd * estVel[iEst];
    BOOST_FOREACH(IndVal& iv, ncorr[iEst])
      dv += (kp * iv.val) * (obsPts[iv.ind] - estPos[iEst]);
    out[iEst] = masses[iEst]*dv;
  }
  return out;
}

CorrPlots::CorrPlots() {
  m_lines.reset(new PlotLines(3));
  m_lines->setDefaultColor(1,1,0,1);
}

void CorrPlots::update(const vector<btVector3>& aPts, const vector<btVector3>& bPts, const SparseArray& corr) {
  vector<btVector3> linePoints;
  for (int iA=0; iA < aPts.size(); iA++) {
    BOOST_FOREACH(const IndVal& iv, corr[iA]) {
      linePoints.push_back(aPts[iA]);
      linePoints.push_back(bPts[iv.ind]);
    }
  }
  m_lines->setPoints(linePoints);
}
