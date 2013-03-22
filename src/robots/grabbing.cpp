#include "grabbing.h"
#include <Eigen/Dense>
#include "simulation/util.h"
#include "utils/config.h"
#include "utils/logging.h"
using namespace std;
using namespace OpenRAVE;
using namespace Eigen;

Grab::Grab(btRigidBody* rb, const btVector3& pos, btDynamicsWorld* world_) {
  world = world_;
  cnt = new btGeneric6DofConstraint(*rb,btTransform::getIdentity(),true); // second parameter?
  cnt->setLinearLowerLimit(btVector3(0,0,0));
  cnt->setLinearUpperLimit(btVector3(0,0,0));
  cnt->setAngularLowerLimit(btVector3(0,0,0));
  cnt->setAngularUpperLimit(btVector3(0,0,0));
  world->addConstraint(cnt);
  updatePosition(pos);
}


Grab::Grab(btRigidBody* rb, const btTransform& pose, btDynamicsWorld* world_) {
  world = world_;
  cnt = new btGeneric6DofConstraint(*rb,rb->getCenterOfMassTransform().inverseTimes(pose),true); // second parameter?
  cnt->setLinearLowerLimit(btVector3(0,0,0));
  cnt->setLinearUpperLimit(btVector3(0,0,0));
  cnt->setAngularLowerLimit(btVector3(0,0,0));
  cnt->setAngularUpperLimit(btVector3(0,0,0));
  world->addConstraint(cnt);
  updatePose(pose);
}

Grab::Grab(btRigidBody* rb, const btTransform& pose,
		btVector3 linLowLim, btVector3 linUpLim,
		btVector3 angLowLim, btVector3 angUpLim,
		btDynamicsWorld* world_) {

	world = world_;
	cnt = new btGeneric6DofConstraint(*rb,rb->getCenterOfMassTransform().inverseTimes(pose),true); // second parameter?
	cnt->setLinearLowerLimit(linLowLim);
	cnt->setLinearUpperLimit(linUpLim);
	cnt->setAngularLowerLimit(angLowLim);
	cnt->setAngularUpperLimit(angUpLim);
	world->addConstraint(cnt);
	updatePose(pose);
}


void Grab::updatePosition(const btVector3& pos) {
  cnt->getFrameOffsetA().setOrigin(pos);
}


void Grab::updatePose(const btTransform& tf) {
  cnt->getFrameOffsetA() = tf;
}

Grab::~Grab() {
  world->removeConstraint(cnt);
  delete cnt;
}



static bool isClosed(RaveRobotObject::Manipulator::Ptr manip, float closedThreshold) {
  return manip->getGripperAngle() < closedThreshold;
}

BulletObject::Ptr getNearestBody(const vector<BulletObject::Ptr>& bodies, const btVector3& pos, int& argmin) {
  assert(bodies.size() > 0);
  VectorXf dists(bodies.size());
  for (int i=0; i < bodies.size(); i++) dists[i] = (bodies[i]->rigidBody->getCenterOfMassPosition() - pos).length();
  dists.minCoeff(&argmin);
  return bodies[argmin];
}

Monitor::Monitor() : closedThreshold(.2) {}

Monitor::Monitor(RaveRobotObject::Manipulator::Ptr manip) :
  m_manip(manip),
  closedThreshold(.2),
  m_wasClosed(isClosed(manip, .2))
{
  cout << "monitor init: " << m_wasClosed << " " << this->isClosed(manip, closedThreshold) << endl;
}

void Monitor::update() {
  bool nowClosed = isClosed(m_manip, closedThreshold);
  if (nowClosed && !m_wasClosed) grab();
  else if (m_wasClosed && !nowClosed) release();
  else if (m_wasClosed && nowClosed) updateGrabPose();
  m_wasClosed = nowClosed;
}

void Monitor::setManip(RaveRobotObject::Manipulator::Ptr m) {
  m_manip = m;
  m_wasClosed = isClosed(m_manip, closedThreshold);
}

MonitorForGrabbing::MonitorForGrabbing(RaveRobotObject::Manipulator::Ptr manip, btDynamicsWorld *dynamicsWorld) :
  Monitor(manip),
  m_world(dynamicsWorld),
  m_bodies(),
  m_grab(NULL),
  m_i(-1)
{
}

void MonitorForGrabbing::setBodies(vector<BulletObject::Ptr>& bodies) {m_bodies = bodies;}

btRigidBody* getNearestDynamicBody(btDynamicsWorld* world, btVector3& pt) {
  btCollisionObjectArray& rigidobjs = world->getCollisionObjectArray();


  float bestDist = SIMD_INFINITY;
  float bestInd = -1;
  bool bestIsRigid = true;

  for (int i = 0; i < rigidobjs.size(); ++i) {
    btRigidBody* maybeRB = btRigidBody::upcast(rigidobjs[i]);
    if (maybeRB && maybeRB->getInvMass() != 0) {
      float dist = maybeRB->getCenterOfMassPosition().distance(pt);
      if (dist < bestDist) {
        bestDist = dist;
        bestInd = i;
      }
    }
  }

  assert (bestInd != -1);
  return btRigidBody::upcast(rigidobjs[bestInd]);
}

void MonitorForGrabbing::grab() {
  // grabs nearest object
  cout << "grabbing nearest object" << endl;
  btTransform curPose = m_manip->getTransform();
  btRigidBody* nearestBody = getNearestDynamicBody(m_world, curPose.getOrigin());
  if (nearestBody->getCenterOfMassPosition().distance(curPose.getOrigin()) < .1*METERS) {
    LOG_INFO("object is close enough: grabbing");
    m_grab = new Grab(nearestBody, curPose, m_world);
//    nearestObj->setColor(0,0,1,1);
  }
  else {
    LOG_INFO("object is too far away");
  }

}

void MonitorForGrabbing::release() {
  cout << "releasing object" << endl;
  if (m_grab != NULL) {
    delete m_grab;
    m_grab = NULL;
    m_i = -1;
  }
}

void MonitorForGrabbing::updateGrabPose() {
  if (!m_grab) return;
  m_grab->updatePose(m_manip->getTransform());
}
