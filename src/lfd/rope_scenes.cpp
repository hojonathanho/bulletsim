#include "knots.h"
#include "rope_scenes.h"
#include "robots/ros2rave.h"
#include <boost/bind.hpp>
#include "robots/pr2.h"
#include "utils/conversions.h"
#include "utils/vector_io.h"

fs::path KNOT_DATA  = fs::path(EXPAND(BULLETSIM_DATA_DIR) "/knots");

static const char LEFT_GRIPPER_LEFT_FINGER_NAME[] = "l_gripper_l_finger_tip_link";
static const char LEFT_GRIPPER_RIGHT_FINGER_NAME[] = "l_gripper_r_finger_tip_link";
static const char RIGHT_GRIPPER_LEFT_FINGER_NAME[] = "r_gripper_l_finger_tip_link";
static const char RIGHT_GRIPPER_RIGHT_FINGER_NAME[] = "r_gripper_r_finger_tip_link";

static inline btTransform getManipRot(RaveRobotObject::Manipulator::Ptr manip) {
  btTransform trans(manip->getTransform());
  trans.setOrigin(btVector3(0, 0, 0));
  return trans;
}
// Returns the direction that the specified finger will move when closing
// (manipulator frame)
static inline btVector3 getClosingDirection(RaveRobotObject::Manipulator::Ptr manip, bool left) {
  // points straight down in the PR2 initial position (manipulator frame)
  btVector3 toolDir = util::toBtVector(manip->manip->GetLocalToolDirection());
  // vector normal to the direction that the gripper fingers move in the manipulator frame
  // (on the PR2 this points back into the arm)
  btVector3 closingNormal(-1, 0, 0);
  return (left ? 1 : -1) * toolDir.cross(closingNormal);
}
// Finds some innermost point on the gripper
static inline btVector3 getInnerPt(RaveRobotObject::Manipulator::Ptr manip, KinBody::LinkPtr leftFinger, KinBody::LinkPtr rightFinger, bool left) {
  btTransform trans(manip->robot->getLinkTransform(left ? leftFinger : rightFinger));
  // we get an innermost point on the gripper by transforming a point
  // on the center of the gripper when it is closed
  return trans * (METERS/20.*btVector3(0.234402, (left ? 1 : -1) * -0.299, 0));
}

// Returns true is pt is on the inner side of the specified finger of the gripper
static inline bool onInnerSide(RaveRobotObject::Manipulator::Ptr manip, const btVector3 &pt, KinBody::LinkPtr leftFinger, KinBody::LinkPtr rightFinger, bool left) {
  // then the innerPt and the closing direction define the plane
  return (getManipRot(manip) * getClosingDirection(manip, left)).dot(pt - getInnerPt(manip, leftFinger, rightFinger, left)) > 0;
}

static bool inGraspRegion(RaveRobotObject::Manipulator::Ptr manip, const btVector3 &pt, KinBody::LinkPtr leftFinger, KinBody::LinkPtr rightFinger) {
  // extra padding for more anchors (for stability)
  static const float TOLERANCE = 0.02;

  // check that pt is behind the gripper tip
  btVector3 x = manip->getTransform().inverse() * pt;
  if (x.z() > GeneralConfig::scale*(0.02 + TOLERANCE)) return false;

  // check that pt is within the finger width
  if (abs(x.x()) > GeneralConfig::scale*(0.01 + TOLERANCE)) return false;

  // check that pt is between the fingers
  if (!onInnerSide(manip, pt, leftFinger, rightFinger, true) ||
      !onInnerSide(manip, pt, leftFinger, rightFinger, false))
    return false;

  return true;
}

static BulletConstraint::Ptr makeGrabConstraint(
    RaveRobotObject::Manipulator::Ptr manip,
    BulletObject::Ptr target) {
  btRigidBody *rb = manip->robot->associatedObj(manip->manip->GetEndEffector())->rigidBody.get();
  btPoint2PointConstraint *c = new btPoint2PointConstraint(
    *rb,
    *target->rigidBody,
    rb->getCenterOfMassTransform().inverse() * target->rigidBody->getCenterOfMassTransform().getOrigin(),
    btVector3(0, 0, 0)
  );
  return BulletConstraint::Ptr(new BulletConstraint(c));
}

void MonitorForGrabbingWithTelekinesis::grab() {
  // grabs nearest object
  cout << "grabbing nearest object" << endl;
  btTransform curPose = m_telekinesis ? m_telePose : m_manip->getTransform();

  int iNear = -1;
  BulletObject::Ptr nearestObj = getNearestBody(m_bodies, curPose.getOrigin(), iNear);
  cout << "grab: " << iNear << endl;

  const int adj_grab = 1;

  if (inGraspRegion(m_manip, nearestObj->rigidBody->getCenterOfMassPosition(), m_leftFinger, m_rightFinger)) {
    cout << "grab success!" << endl;
    for (int i = iNear-adj_grab; i <= iNear+adj_grab; ++i) {
      if (i < 0 || i >= m_bodies.size()) continue;
      BulletConstraint::Ptr grab = makeGrabConstraint(m_manip, m_bodies[i]);
      m_grabs.push_back(grab);
      m_env->addConstraint(grab);
      m_bodies[i]->setColor(0,0,1,1);
      cout << "CONSTRAINTS AFTER GRABBING: " << m_env->constraints.size() << endl;
    }
  }
}

void MonitorForGrabbingWithTelekinesis::release() {
  for (int i = 0; i < m_grabs.size(); ++i) {
    m_env->removeConstraint(m_grabs[i]);
    m_grabs[i].reset();
  }
  m_grabs.clear();
}

void MonitorForGrabbingWithTelekinesis::updateGrabPose() {
  if (!m_grab) return;
  btTransform curPose = m_telekinesis ? m_telePose : m_manip->getTransform();
//  m_grab->updatePose(curPose);
}

GrabbingScene::GrabbingScene(bool telekinesis)  {
  pr2m.reset(new PR2Manager(*this));
  m_lMonitor.reset(new MonitorForGrabbingWithTelekinesis(pr2m->pr2Left, env,telekinesis, pr2m->pr2->robot->GetLink(LEFT_GRIPPER_LEFT_FINGER_NAME), pr2m->pr2->robot->GetLink(LEFT_GRIPPER_RIGHT_FINGER_NAME)));
  m_rMonitor.reset(new MonitorForGrabbingWithTelekinesis(pr2m->pr2Right, env, telekinesis, pr2m->pr2->robot->GetLink(RIGHT_GRIPPER_LEFT_FINGER_NAME), pr2m->pr2->robot->GetLink(RIGHT_GRIPPER_RIGHT_FINGER_NAME)));
  pr2m->setHapticCb(hapticLeft0Hold, boost::bind(&GrabbingScene::closeLeft, this));
  pr2m->setHapticCb(hapticLeft1Hold, boost::bind(&GrabbingScene::openLeft, this));
  pr2m->setHapticCb(hapticRight0Hold, boost::bind(&GrabbingScene::closeRight, this));
  pr2m->setHapticCb(hapticRight1Hold, boost::bind(&GrabbingScene::openRight, this));
  if (telekinesis) {
    pr2m->armsDisabled = true;
    m_teleLeft.reset(new TelekineticGripper(pr2m->pr2Left));
    m_teleRight.reset(new TelekineticGripper(pr2m->pr2Right));
    env->add(m_teleLeft);
    env->add(m_teleRight);
  }

  float step = .01;
  VoidCallback cb = boost::bind(&GrabbingScene::drive, this, step, 0);
  addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Left, boost::bind(&GrabbingScene::drive, this, 0, step));
  addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Right, boost::bind(&GrabbingScene::drive, this, 0, -step));
  addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Up, boost::bind(&GrabbingScene::drive, this, -step, 0));
  addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Down, boost::bind(&GrabbingScene::drive, this, step, 0));
}

void GrabbingScene::step(float dt) {
//  m_lMonitor->updateGrabPose();
//  m_rMonitor->updateGrabPose();

  Scene::step(dt);
}

BulletObject::Ptr makeTable(const vector<btVector3>& corners, float thickness) {
  btVector3 origin = (corners[0] + corners[2])/2;
  origin[2] -= thickness/2;
  btVector3 halfExtents = (corners[2] - corners[0]).absolute()/2;
  halfExtents[2] = thickness/2;

  return BulletObject::Ptr(new BoxObject(0,halfExtents,btTransform(btQuaternion(0,0,0,1),origin)));
}

TableRopeScene::TableRopeScene(const vector<btVector3> &tableCornersWorld_, const vector<btVector3>& controlPointsWorld, bool telekinesis) : GrabbingScene(telekinesis), tableCornersWorld(tableCornersWorld_) {
  vector<int> indices(1,pr2m->pr2->robot->GetJointIndex("torso_lift_joint"));
  vector<double> values(1, .31);
  pr2m->pr2->setDOFValues(indices, values);
  PlotPoints::Ptr corners(new PlotPoints(20));
  corners->setPoints(tableCornersWorld);
  env->add(corners);

  m_table = makeTable(tableCornersWorld, .1*GeneralConfig::scale);
  float seglen = controlPointsWorld[0].distance(controlPointsWorld[1]);

  resetRope(controlPointsWorld);
  env->add(m_table);
}

void TableRopeScene::resetRope(const vector<btVector3> &ctrlPoints) {
  if (m_rope) {
    m_lMonitor->release();
    m_rMonitor->release();
    env->remove(m_rope);
  }
  m_rope.reset(new CapsuleRope(
    ctrlPoints,
    .005*METERS, // radius
    .5, // angStiffness
    1, // angDamping
    .9, // linDamping
    .8, // angLimit
    .9 // linStopErp
  ));
  env->add(m_rope);
  setGrabBodies(m_rope->children);
}

vector<btVector3> operator*(const vector<btVector3>& in, float a) {
  vector<btVector3> out(in.size());
  for (int i=0; i < in.size(); i++) {
    out[i] = in[i] * a;
  }
  return out;
}

btTransform operator*(const btTransform& in, float a) {
  btTransform out;
  out.setOrigin(in.getOrigin() * a);
  out.setRotation(in.getRotation());
  return out;
}
