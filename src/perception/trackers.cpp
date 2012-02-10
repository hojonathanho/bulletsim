#include "trackers.h"
#include "matching.h"
#include "dist_math.h"
#include "utils_perception.h"
#include <boost/foreach.hpp>
#include "utils/config.h"
#include "utils/vector_io.h"

using namespace Eigen;

MultiPointTrackerRigid::MultiPointTrackerRigid(vector< RigidBodyPtr >& bodies, btDynamicsWorld* world)
  : m_N(bodies.size()), m_bodies(bodies), m_active(bodies.size(), false), m_world(world) {
  BOOST_FOREACH(RigidBodyPtr body, bodies)
    m_constraints.push_back(P2PConstraintPtr(new btPoint2PointConstraint(*body.get(), btVector3(0,0,0))));
}

void MultiPointTrackerRigid::update(const vector<btVector3>& obsPts) {
  vector<btVector3> estPts(m_N);
  for (int iEst=0; iEst < m_N; iEst++) estPts[iEst] = m_bodies[iEst]->getCenterOfMassPosition();
  Eigen::MatrixXf costs = pairwiseSquareDist(toEigenMatrix(estPts), toEigenMatrix(obsPts));
  VectorXi matches = matchHard(costs, .3*METERS*.3*METERS); // don't bother matching more than a foot
  cout << costs << endl;
  cout << matches << endl;
  for (int iEst=0; iEst < m_N; iEst++) {
    if (matches[iEst] != -1) {
      updatePos(iEst, obsPts[matches[iEst]]);
      setActive(iEst,true);
    }
    else  setActive(iEst,false);
  }
}

void MultiPointTrackerRigid::setActive(int iBody, bool isActive) {
  if (m_active[iBody] && !isActive) m_world->removeConstraint(m_constraints[iBody].get());
  if (!m_active[iBody] && isActive) m_world->addConstraint(m_constraints[iBody].get());
  m_active[iBody] = isActive;
};

void MultiPointTrackerRigid::updatePos(int iBody, const btVector3& newPos) {
  m_constraints[iBody]->setPivotB(newPos);
}

TrackerPlotter::TrackerPlotter(MultiPointTrackerRigid& tracker) : m_tracker(&tracker), m_N(tracker.m_N) {
  for (int i=0; i < m_tracker->m_N; i++) {
    btVector3 curPos = m_tracker->m_bodies[i]->getCenterOfMassPosition();
    MotionStatePtr ms(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),curPos)));
    BulletObject::Ptr fakeObject(new SphereObject(0,.015*METERS, ms));
    fakeObject->rigidBody->setCollisionFlags(btRigidBody::CF_NO_CONTACT_RESPONSE);
    fakeObject->rigidBody->setActivationState(DISABLE_DEACTIVATION);
    m_fakeObjects.push_back(fakeObject);
  }
}

void TrackerPlotter::update() {
  vector<btVector3> positions(m_N);
  for (int i=0; i < m_N; i++) {
    m_fakeObjects[i]->rigidBody->setCenterOfMassTransform(m_tracker->m_bodies[i]->getCenterOfMassTransform());
    if (m_tracker->m_active[i]) m_fakeObjects[i]->setColor(0,0,1,.25);
    else m_fakeObjects[i]->setColor(1,1,1,.25);
  }
}
