#include "tracked_compound.h"
#include "config_tracking.h"
#include "utils/conversions.h"
#include "utils/cvmat.h"
#include "simulation/bullet_io.h"
#include "utils/utils_vector.h"
#include "tracking/surface_sampling.h"
#include <boost/foreach.hpp>
#include "tracking/feature_extractor.h"

using namespace std;
using namespace Eigen;

TrackedCompound::TrackedCompound(GenericCompoundObject::Ptr sim, btDynamicsWorld* world) : TrackedObject(sim, "compound") {
  m_nBodies = sim->children.size();
  VectorXf bodyMasses(m_nBodies);
  for (int i=0; i < m_nBodies; i++) {
	sim->children[i];
	sim->children[i]->rigidBody;
    bodyMasses(i) = sim->children[i]->rigidBody->getInvMass();
  }

  if (world == NULL) {
  	Environment::Ptr env = sim->getEnvironment();
  	assert(env);
 		world = env->bullet->dynamicsWorld;
  }

  getSampledDescription(sim, world, TrackingConfig::downsample*METERS, m_node2body, m_relativePos);
  m_nNodes = m_node2body.size();
  for (int i=0; i < m_nNodes; ++i) {
	  m_body2nodes[m_node2body[i]].push_back(i);
  }

  m_masses.resize(m_nNodes);
  for (int i=0; i < m_nBodies; ++i) {
	  vector<int>& nodes = m_body2nodes[i];
	  BOOST_FOREACH(int node, nodes) m_masses(node) = bodyMasses(i) / nodes.size();
  }

  init();
}

std::vector<btVector3> TrackedCompound::getPoints() {
	vector<btTransform> transforms;
	transforms.reserve(m_nBodies);
	BOOST_FOREACH(BulletObject::Ptr& child, getSim()->children) transforms.push_back(child->rigidBody->getCenterOfMassTransform());

	vector<btVector3> out(m_nNodes);
	for (int i=0; i < m_nNodes; ++i) {
		int bodyInd = m_node2body[i];
		out[i] = transforms[bodyInd] * m_relativePos[i];
	}
	return out;
}

void TrackedCompound::applyEvidence(const Eigen::MatrixXf& corr, const MatrixXf& obsPts) {
	vector<btTransform> transforms;
	transforms.reserve(m_nBodies);
	vector<btRigidBody*> bodies;
	bodies.reserve(m_nBodies);
	BOOST_FOREACH(BulletObject::Ptr& child, getSim()->children) {
		transforms.push_back(child->rigidBody->getCenterOfMassTransform());
		bodies.push_back(child->rigidBody.get());
	}
	vector<btVector3> pos(m_nNodes);
	vector<btVector3> vel(m_nNodes);
	for (int i=0; i < m_nNodes; ++i) {
		int bodyInd = m_node2body[i];
		pos[i] = transforms[bodyInd] * m_relativePos[i];
		vel[i] = bodies[bodyInd]->getVelocityInLocalPoint(m_relativePos[i]);
	}
  vector<btVector3> impulses = calcImpulsesDamped(pos, vel, toBulletVectors(obsPts.leftCols(3)), corr, toVec(m_masses), TrackingConfig::kp_rope, TrackingConfig::kd_rope);

	for (int i=0; i<m_nNodes; ++i) {
	  getSim()->children[m_node2body[i]]->rigidBody->applyImpulse(impulses[i], m_relativePos[i]);
	}

}

void TrackedCompound::initColors() {
	TrackedObject::initColors();
	return;
	for (int i=0; i < m_nBodies; ++i) {
		Vector3f bgr = toEigenMatrixImage(getSim()->children[i]->getTexture()).colwise().mean();
		m_colors.row(i) = bgr.transpose();
	}
}
