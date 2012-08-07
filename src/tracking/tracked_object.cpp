#include "tracked_object.h"
#include "utils/conversions.h"
#include "config_tracking.h"
#include "utils/utils_vector.h"

using namespace std;

TrackedObject::TrackedObject(EnvironmentObject::Ptr sim, string type) : m_sim(sim), m_type(type) { }

void TrackedObject::init() {
	initColors();
}

Eigen::MatrixXf& TrackedObject::getColors() {
	return m_colors;
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
  	for (int n=0; n<N; n++)
			dv += (kp * corr(k,n)) * (obsPts[n] - estPos[k]);
		impulses[k] = masses[k]*dv;
		// XXX SHOULD THERE BE DT?
  }
  return impulses;
}
