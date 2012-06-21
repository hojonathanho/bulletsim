#include "tracked_object.h"
#include "config_tracking.h"
#include "utils/conversions.h"
#include "simulation/bullet_io.h"

using namespace std;
using namespace Eigen;

TrackedRope::TrackedRope(CapsuleRope::Ptr sim) : TrackedObject(sim, "rope") {
  m_nNodes = sim->children.size();
  m_masses.resize(m_nNodes);
  for (int i=0; i < m_nNodes; i++) {
    m_masses(i) = 1/getSim()->children[i]->rigidBody->getInvMass();
  }
}

std::vector<btVector3> TrackedRope::getPoints() {
	std::vector<btVector3> out(m_nNodes);
	for (int i=0; i < m_nNodes; ++i) {
		out[i] = getSim()->children[i]->rigidBody->getCenterOfMassPosition();
	}
	return out;
}

MatrixXf TrackedRope::getFeatures() {
	MatrixXf features(m_nNodes, 6);
	for (int i=0; i < m_nNodes; i++) {
		features.block(i,0,1,3) = toEigenVector(getSim()->children[i]->rigidBody->getCenterOfMassPosition()).transpose();
		features(i,3) = getSim()->children[i]->getTexture().at<cv::Vec3b>(0,0)[2]; //R
		features(i,4) = getSim()->children[i]->getTexture().at<cv::Vec3b>(0,0)[1]; //G
		features(i,5) = getSim()->children[i]->getTexture().at<cv::Vec3b>(0,0)[0]; //B
	}
	return features;
}

void TrackedRope::applyEvidence(const SparseMatrixf& corr, const MatrixXf& obsPts) {
  vector<btVector3> estPos(m_nNodes), estVel(m_nNodes);
  for (int i=0; i < m_nNodes; ++i)  {
    estPos[i] = getSim()->children[i]->rigidBody->getCenterOfMassPosition();
    estVel[i] = getSim()->children[i]->rigidBody->getLinearVelocity();
  }
  vector<btVector3> impulses = calcImpulsesDamped(estPos, estVel, toBulletVectors(obsPts.block(0,0,obsPts.rows(),3)), corr, toVec(m_masses), TrackingConfig::kp_rope, TrackingConfig::kd_rope);

  for (int i=0; i<m_nNodes; ++i) getSim()->children[i]->rigidBody->applyCentralImpulse(impulses[i]);

  //TODO apply evidences to color
}
