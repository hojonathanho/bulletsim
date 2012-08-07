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

std::vector<btVector3> calcImpulsesDamped(const std::vector<btVector3>& estPos, const std::vector<btVector3>& estVel,
  const std::vector<btVector3>& obsPts, const Eigen::MatrixXf& corr, const vector<float>& masses, float kp, float kd) {
  int K = estPos.size();
  int N = obsPts.size();
  assert(estVel.size() == K);
  assert(corr.rows() == K);
  assert(corr.cols() == N);
  assert(masses.size() == K);
  vector<btVector3> impulses(K);

  for (int k=0; k<K; k++) {
  	btVector3 dv = -kd * estVel[k];
  	//(kp * corr.row(k)) * (obsPts - estPos[k]);
  	for (int n=0; n<N; n++)
			dv += (kp * corr(k,n)) * (obsPts[n] - estPos[k]);
		impulses[k] = masses[k]*dv;
		// XXX SHOULD THERE BE DT?
  }
  return impulses;
}
