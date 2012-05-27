#include "tracked_object.h"
using namespace std;


std::vector<btVector3> calcImpulsesDamped(const std::vector<btVector3>& estPos, const std::vector<btVector3>& estVel, 
  const std::vector<btVector3>& obsPts, const SparseMatrixf& corr, const vector<float>& masses, float kp, float kd) {
  int nEst = estPos.size();
  int nObs = obsPts.size();
  vector<btVector3> out(nEst);

  for (int iEst=0; iEst<corr.rows(); ++iEst) {
    btVector3 dv = -kd * estVel[iEst];
    for (SparseMatrixf::InnerIterator it(corr,iEst); it; ++it)
    {
      dv += (kp * it.value()) * (obsPts[it.col()] - estPos[iEst]);
    } 
    out[iEst] = masses[iEst]*dv;
    // XXX SHOULD THERE BE DT?
  }

  return out;
}
