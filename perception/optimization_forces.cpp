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
  SparseArray out(nEst);
  MatrixXf distsEstObs = pairwiseSquareDist(toEigenMatrix(estPts), toEigenMatrix(obsPts));
  vector<int> estToObs = argminAlongRows(distsEstObs);
  vector<int> obsToEst = argminAlongRows(distsEstObs.transpose());
  for (int iEst=0; iEst<nEst; iEst++) 
    if (pVis[iEst] > .5) 
      out[iEst].push_back(IndVal(estToObs[iEst],2));
  for (int iObs=0; iObs<nObs; iObs++) out[obsToEst[iObs]].push_back(IndVal(iObs,1));
  return out;
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

SparseArray normalizeRows(const SparseArray& in) {
  SparseArray out(in.size());
  for (int iRow=0; iRow < in.size(); iRow++) {
    SparseVector inRow = in[iRow];
    SparseVector outRow = out[iRow];
    outRow.reserve(inRow.size());
    float rowSum = vecSum(inRow);
    BOOST_FOREACH(IndVal& iv, inRow) outRow.push_back(IndVal(iv.ind,iv.val/rowSum));
  }
  return out;
}

CorrPlots::CorrPlots() {
  m_lines.reset(new PlotLines(3));
  m_lines->setDefaultColor(1,1,0,1);
}

void CorrPlots::update(const vector<btVector3>& aPts, const vector<btVector3>& bPts, const SparseArray& corr) {

  vector<btVector3> lineStarts;
  vector<btVector3> lineEnds;
  for (int iA=0; iA < aPts.size(); iA++) {
    BOOST_FOREACH(const IndVal& iv, corr[iA]) {
      lineStarts.push_back(aPts[iA]);
      lineEnds.push_back(bPts[iv.ind]);
    }
  }
  m_lines->setPoints(lineStarts,lineEnds);
}
