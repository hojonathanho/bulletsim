#include "tracked_object.h"
#include "config_tracking.h"
#include "utils/conversions.h"
#include "utils/cvmat.h"
#include "simulation/bullet_io.h"
#include "utils/utils_vector.h"
#include "feature_extractor.h"

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

void TrackedRope::applyEvidence(const Eigen::MatrixXf& corr, const MatrixXf& obsPts) {
  vector<btVector3> estPos(m_nNodes), estVel(m_nNodes);
  for (int i=0; i < m_nNodes; ++i)  {
    estPos[i] = getSim()->children[i]->rigidBody->getCenterOfMassPosition();
    estVel[i] = getSim()->children[i]->rigidBody->getLinearVelocity();
  }
  vector<btVector3> impulses = calcImpulsesDamped(estPos, estVel, toBulletVectors(FE::activeFeatures2Feature(obsPts, FE::FT_XYZ)), corr, toVec(m_masses), TrackingConfig::kp_rope, TrackingConfig::kd_rope);

  for (int i=0; i<m_nNodes; ++i) getSim()->children[i]->rigidBody->applyCentralImpulse(impulses[i]);
}

void TrackedRope::initColors() {
	m_colors.resize(m_nNodes, 3);
	for (int i=0; i < m_nNodes; ++i) {
		Vector3f bgr = toEigenMatrixImage(getSim()->children[i]->getTexture()).colwise().mean();
		m_colors.row(i) = bgr.transpose();
	}
}
