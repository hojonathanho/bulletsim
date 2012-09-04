#ifndef _LFD_ROPE_SCENES_H_
#define _LFD_ROPE_SCENES_H_

#include "simulation/simplescene.h"
#include "simulation/rope.h"
#include "robots/grabbing.h"
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;


struct MonitorForGrabbingWithTelekinesis : public MonitorForGrabbing {
  btTransform m_telePose;
  bool m_telekinesis;
  KinBody::LinkPtr m_leftFinger, m_rightFinger;

  MonitorForGrabbingWithTelekinesis(RaveRobotObject::Manipulator::Ptr manip, btDynamicsWorld* dynamicsWorld, bool telekinesis, KinBody::LinkPtr leftFinger, KinBody::LinkPtr rightFinger)
    : m_telekinesis(telekinesis), m_telePose(btTransform::getIdentity()), MonitorForGrabbing(manip, dynamicsWorld), m_leftFinger(leftFinger), m_rightFinger(rightFinger) {}

  void grab();

  void updateGrabPose();

};




struct GrabbingScene : public Scene {
public:
  boost::shared_ptr<PR2Manager> pr2m;
  boost::shared_ptr<MonitorForGrabbingWithTelekinesis> m_lMonitor, m_rMonitor;
  TelekineticGripper::Ptr m_teleLeft, m_teleRight;

  GrabbingScene(bool telekinesis=false);

  void step(float dt);

  void setGrabBodies(vector<BulletObject::Ptr> bodies) {
    m_lMonitor->setBodies(bodies);
    m_rMonitor->setBodies(bodies);
  }

  void closeLeft() {
    pr2m->pr2Left->setGripperAngle(pr2m->pr2Left->getGripperAngle() - .02);
  }
  void openLeft() {
    pr2m->pr2Left->setGripperAngle(pr2m->pr2Left->getGripperAngle() + .02);
  }
  void closeRight() {
    pr2m->pr2Right->setGripperAngle(pr2m->pr2Right->getGripperAngle() - .02);
  } 
  void openRight() {
    pr2m->pr2Right->setGripperAngle(pr2m->pr2Right->getGripperAngle() + .02);
  }
  void drive(float dx /*meters*/, float dy /*meters*/) {
    OpenRAVE::Vector rotation(1,0,0,0);
    OpenRAVE::Vector translation(dx,dy,0);
    OpenRAVE::Transform tf(rotation, translation);
    pr2m->pr2->robot->SetTransform(tf*(pr2m->pr2->robot->GetTransform()));
    pr2m->pr2->updateBullet();
  }
  void driveTo(const btTransform& tf /*meters*/) {
    pr2m->pr2->robot->SetTransform(util::toRaveTransform(tf));
    pr2m->pr2->updateBullet();
  }
  void setFakeHandPoses(const btTransform& leftPose, const btTransform& rightPose) {
    setFakeLeft(leftPose);
    setFakeRight(rightPose);
  }
  void setFakeLeft(const btTransform& leftPose) {
    m_lMonitor->m_telePose = leftPose;
    m_teleLeft->setTransform(leftPose);
  }
  void setFakeRight(const btTransform& rightPose) {
    m_rMonitor->m_telePose = rightPose;
    m_teleRight->setTransform(rightPose);
  }

};

std::vector<btVector3> operator*(const std::vector<btVector3>& in, float a);
btTransform operator*(const btTransform& in, float a);

struct TableRopeScene : public GrabbingScene {
  CapsuleRope::Ptr m_rope;
  BulletObject::Ptr m_table;
  const vector<btVector3> tableCornersWorld;

  TableRopeScene(const vector<btVector3> &tableCornersWorld_, const vector<btVector3>& controlPointsWorld, bool telekinesis=false);

  CapsuleRope::Ptr getRope() const { return m_rope; }
};

#endif // _LFD_ROPE_SCENES_H_
