#include "grabbing.h"
#include <Eigen/Dense>
#include "util.h"
#include "config.h"
using namespace std;
using namespace OpenRAVE;
using namespace Eigen;

Grab::Grab(btRigidBody* rb, btVector3 pos, btDynamicsWorld* world_) {
  world = world_;
  cnt = new btGeneric6DofConstraint(*rb,btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)),true); // second parameter?
  cnt->setLinearLowerLimit(btVector3(0,0,0));
  cnt->setLinearUpperLimit(btVector3(0,0,0));
  cnt->setAngularLowerLimit(btVector3(0,0,0));
  cnt->setAngularUpperLimit(btVector3(0,0,0));
  world->addConstraint(cnt);
  updatePosition(pos);
}

void Grab::updatePosition(btVector3 pos) {
  cnt->getFrameOffsetA().setOrigin(pos);
}

Grab::~Grab() {
  world->removeConstraint(cnt);
  delete cnt;
}

const float CLOSED_VAL = 0.03f, OPEN_VAL = 0.54f;
const float MIDDLE_VAL = .28f;

bool isClosed(RobotBase::ManipulatorPtr manip) {
  vector<int> gripperInds = manip->GetGripperIndices();
  manip->GetRobot()->SetActiveDOFs(gripperInds);
  vector<double> dof_values;
  manip->GetRobot()->GetActiveDOFValues(dof_values);
  cout << "gripper joint: " << dof_values[0] << endl;
  return dof_values[0] < MIDDLE_VAL;
}

BulletObject::Ptr getNearestBody(vector<BulletObject::Ptr> bodies, btVector3 pos) {
  VectorXf dists(bodies.size());
  for (int i=0; i < bodies.size(); i++) dists[i] = (bodies[i]->rigidBody->getCenterOfMassPosition() - pos).length();
  int argmin;
  dists.minCoeff(&argmin);
  return bodies[argmin];
}


MonitorForGrabbing::MonitorForGrabbing(RobotBase::ManipulatorPtr manip, BulletInstance::Ptr bullet) :
  Monitor(manip, bullet),
  m_world(bullet->dynamicsWorld),
  m_bodies(),
  m_wasClosed(isClosed(manip)),
  m_grab(NULL)
{}

void MonitorForGrabbing::setBodies(vector<BulletObject::Ptr>& bodies) {m_bodies = bodies;}

void MonitorForGrabbing::update() {
  bool nowClosed = isClosed(manip);

  if (nowClosed && !m_wasClosed) grabNearestObject();
  else if (m_wasClosed && !nowClosed) releaseObject();
  else if (m_wasClosed && nowClosed && m_grab!=NULL) {
    m_grab->updatePosition(util::toBtVector(manip->GetTransform().trans)*METERS);
    cout << "updating constraint position" << endl;
  }

  m_wasClosed = nowClosed;
}

void MonitorForGrabbing::grabNearestObject() {
  cout << "grabbing nearest object" << endl;
  btVector3 curPos = util::toBtVector(manip->GetTransform().trans)*METERS;
  cout << "curPos: " << curPos.x() << " " << curPos.y() << " " << curPos.z() << endl;
  BulletObject::Ptr nearestObj = getNearestBody(m_bodies, curPos);
  m_grab = new Grab(nearestObj->rigidBody.get(), curPos, m_world);
  nearestObj->setColor(0,0,1,1);
}

void MonitorForGrabbing::releaseObject() {
  cout << "releasing object" << endl;
  if (m_grab != NULL) delete m_grab;
}
