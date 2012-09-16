#include "tracked_object.h"
#include "utils/conversions.h"
#include "config_tracking.h"
#include "utils/utils_vector.h"
#include "feature_extractor.h"

using namespace std;

TrackedObject::TrackedObject(EnvironmentObject::Ptr sim, string type) : m_sim(sim), m_type(type) { }

void TrackedObject::init() {
	initColors();
}

Eigen::MatrixXf& TrackedObject::getColors() {
	return m_colors;
}

const Eigen::VectorXf TrackedObject::getPriorDist() {
	Eigen::MatrixXf prior_dist(1,FeatureExtractor::m_allDim);
	prior_dist << TrackingConfig::pointPriorDist*METERS, TrackingConfig::pointPriorDist*METERS, TrackingConfig::pointPriorDist*METERS,  //FT_XYZ
			0.2, 0.2, 0.2, 	//FT_BGR
			TrackingConfig::colorLPriorDist, TrackingConfig::colorABPriorDist, TrackingConfig::colorABPriorDist,	//FT_LAB
			1.0, 1.0, 1.0,  //FT_NORMAL
			1.0,  //FT_LABEL
			Eigen::MatrixXf::Ones(1, FE::FT_SIZES[FE::FT_SURF])*0.4,  //FT_SURF
			Eigen::MatrixXf::Ones(1, FE::FT_SIZES[FE::FT_PCASURF])*0.4,  //FT_PCASURF
			0.5;  //FT_GRADNORMAL
	return FeatureExtractor::all2ActiveFeatures(prior_dist).transpose();
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

//  float max_impulse=1;
//  for (int k=0; k<K; k++) {
//  	if (impulses[k].length2() > max_impulse*max_impulse) {
//  		impulses[k].normalize();
//			impulses[k] *= max_impulse;
//  	}
//  }
//  cout << "max impulse mag " << max(impulses).length() << endl;

  return impulses;
}
