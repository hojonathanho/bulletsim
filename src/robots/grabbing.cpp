#include "grabbing.h"
#include <Eigen/Dense>
#include "simulation/util.h"
#include "utils/config.h"
using namespace std;
using namespace OpenRAVE;
using namespace Eigen;

Grab::Grab(btRigidBody* rb, const btVector3& pos, btDynamicsWorld* world_) {
  world = world_;
  cnt = new btGeneric6DofConstraint(*rb,btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)),true); // second parameter?
  cnt->setLinearLowerLimit(btVector3(0,0,0));
  cnt->setLinearUpperLimit(btVector3(0,0,0));
  cnt->setAngularLowerLimit(btVector3(0,0,0));
  cnt->setAngularUpperLimit(btVector3(0,0,0));
  world->addConstraint(cnt);
  updatePosition(pos);
}

void Grab::updatePosition(const btVector3& pos) {
  cnt->getFrameOffsetA().setOrigin(pos);
}

Grab::~Grab() {
  world->removeConstraint(cnt);
  delete cnt;
}


static bool isClosed(RaveRobotObject::Manipulator::Ptr manip, float closedThreshold) {
  return manip->getGripperAngle() < closedThreshold;
}

static BulletObject::Ptr getNearestBody(vector<BulletObject::Ptr> bodies, btVector3 pos, int argmin) {
  VectorXf dists(bodies.size());
  for (int i=0; i < bodies.size(); i++) dists[i] = (bodies[i]->rigidBody->getCenterOfMassPosition() - pos).length();
  dists.minCoeff(&argmin);
  return bodies[argmin];
}

Monitor::Monitor() : closedThreshold(PR2_CLOSED_VAL) {}

Monitor::Monitor(RaveRobotObject::Manipulator::Ptr manip) :
    m_manip(manip),
    closedThreshold(PR2_CLOSED_VAL),
    m_wasClosed(isClosed(manip, PR2_CLOSED_VAL))
{
  cout << "monitor init: " << m_wasClosed << " " << isClosed(manip, closedThreshold) << endl;
}

void Monitor::update() {
  bool nowClosed = isClosed(m_manip, closedThreshold);
  if (nowClosed && !m_wasClosed) grab();
  else if (m_wasClosed && !nowClosed) release();
  else if (m_wasClosed && nowClosed) updateGrabPos();
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

void MonitorForGrabbing::grab() {
  // grabs nearest object
  cout << "grabbing nearest object" << endl;
  btVector3 curPos = m_manip->getTransform().getOrigin();
  cout << "curPos: " << curPos.x() << " " << curPos.y() << " " << curPos.z() << endl;
  BulletObject::Ptr nearestObj = getNearestBody(m_bodies, curPos, m_i);
  m_grab = new Grab(nearestObj->rigidBody.get(), curPos, m_world);
  nearestObj->setColor(0,0,1,1);
}

void MonitorForGrabbing::release() {
  cout << "releasing object" << endl;
  if (m_grab != NULL) {
    delete m_grab;
    m_grab = NULL;
    m_i = -1;
  }
}

void MonitorForGrabbing::updateGrabPos() {
    if (!m_grab) return;
    m_grab->updatePosition(m_manip->getTransform().getOrigin());
}
