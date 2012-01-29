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

static bool isClosed(RobotBase::ManipulatorPtr manip) {
  vector<int> gripperInds = manip->GetGripperIndices();
  manip->GetRobot()->SetActiveDOFs(gripperInds);
  vector<double> dof_values;
  manip->GetRobot()->GetActiveDOFValues(dof_values);
  cout << "gripper joint: " << dof_values[0] << endl;
  return dof_values[0] < MIDDLE_VAL;
}

static BulletObject::Ptr getNearestBody(vector<BulletObject::Ptr> bodies, btVector3 pos) {
  VectorXf dists(bodies.size());
  for (int i=0; i < bodies.size(); i++) dists[i] = (bodies[i]->rigidBody->getCenterOfMassPosition() - pos).length();
  int argmin;
  dists.minCoeff(&argmin);
  return bodies[argmin];
}

Monitor::Monitor(OpenRAVE::RobotBase::ManipulatorPtr manip) :
    m_manip(manip),
    m_wasClosed(isClosed(manip))
{
}

void Monitor::update() {
  bool nowClosed = isClosed(m_manip);
  if (nowClosed && !m_wasClosed) grab();
  else if (m_wasClosed && !nowClosed) release();
  else if (m_wasClosed && nowClosed) updateGrabPos();
  m_wasClosed = nowClosed;
}

MonitorForGrabbing::MonitorForGrabbing(OpenRAVE::RobotBase::ManipulatorPtr manip, btDynamicsWorld *dynamicsWorld) :
  Monitor(manip),
  m_world(dynamicsWorld),
  m_bodies(),
  m_grab(NULL)
{
}

void MonitorForGrabbing::setBodies(vector<BulletObject::Ptr>& bodies) {m_bodies = bodies;}

void MonitorForGrabbing::grab() {
  // grabs nearest object
  cout << "grabbing nearest object" << endl;
  btVector3 curPos = util::toBtVector(m_manip->GetTransform().trans)*METERS;
  cout << "curPos: " << curPos.x() << " " << curPos.y() << " " << curPos.z() << endl;
  BulletObject::Ptr nearestObj = getNearestBody(m_bodies, curPos);
  m_grab = new Grab(nearestObj->rigidBody.get(), curPos, m_world);
  nearestObj->setColor(0,0,1,1);
}

void MonitorForGrabbing::release() {
  cout << "releasing object" << endl;
  if (m_grab != NULL) delete m_grab;
}

void MonitorForGrabbing::updateGrabPos() {
    if (!m_grab) return;
    cout << "updating constraint position" << endl;
    m_grab->updatePosition(util::toBtVector(m_manip->GetTransform().trans)*METERS);
}
