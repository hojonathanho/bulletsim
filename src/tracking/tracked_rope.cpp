#include "tracked_object.h"
#include "config_tracking.h"
#include "utils/conversions.h"

using namespace std;
using namespace Eigen;

TrackedRope::TrackedRope(CapsuleRope::Ptr sim) : TrackedObject(m_sim) {
  m_nNodes = sim->children.size();
  m_masses.resize(m_nNodes);
  for (int i=0; i < m_nNodes; i++) {
    m_masses(i) = 1/getSim()->children[i]->rigidBody->getInvMass();
  }
}

Eigen::MatrixXf TrackedRope::getPoints() {
	Eigen::Matrix3Xf out(m_nNodes, 3);
	for (int i=0; i < m_nNodes; ++i) {
		out.row(i) = toEigenVector(getSim()->children[i]->rigidBody->getCenterOfMassPosition());
	}
	return out;
}

void TrackedRope::applyEvidence(const SparseMatrixf& corr, const MatrixXf& obsPts) {


  vector<btVector3> estPos, estVel;
  estPos.reserve(m_nNodes);
  estVel.reserve(m_nNodes);
  for (int i=0; i < m_nNodes; ++i)  {
    estPos[i] = getSim()->children[i]->rigidBody->getCenterOfMassPosition();
    estVel[i] = getSim()->children[i]->rigidBody->getLinearVelocity();
  }
  vector<btVector3> impulses = calcImpulsesDamped(estPos, estVel, toBulletVectors(obsPts), corr, toVec(m_masses), TrackingConfig::kp_rope, TrackingConfig::kd_rope);
  //m_sigs = 1*VectorXf::Ones(ropePos.size());

  for (int i=0; i<m_nNodes; ++i) getSim()->children[i]->rigidBody->applyCentralImpulse(impulses[i]);
}
