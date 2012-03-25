#pragma once
#include "simulation/simplescene.h"
#include "robots/grabbing.h"
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include "utils/vector_io.h"
#include "robots/pr2.h"
#include "simulation/rope.h"
#include "utils/conversions.h"

using boost::shared_ptr;
namespace fs = boost::filesystem;

extern fs::path KNOT_DATA;

static const int N_CTRL_PTS = 100;

struct RobotState{
  btTransform leftPose, rightPose;
  float leftGrip, rightGrip;
};

struct RobotAndRopeState {
  btTransform leftPose, rightPose;
  float leftGrip, rightGrip;
  int leftGrab, rightGrab;
  vector<btVector3> ctrlPts;
};

struct MonitorForGrabbingWithTelepathy : public MonitorForGrabbing {
  btTransform m_telePose;
  bool m_telepathy;

  MonitorForGrabbingWithTelepathy(RaveRobotObject::Manipulator::Ptr manip, btDynamicsWorld* dynamicsWorld, bool telepathy)
    : m_telepathy(telepathy), m_telePose(btTransform::getIdentity()), MonitorForGrabbing(manip, dynamicsWorld) {}

  void grab();

  void updateGrabPose();

};


struct GrabbingScene : public Scene {
public:
  shared_ptr<PR2Manager> pr2m;
  shared_ptr<MonitorForGrabbingWithTelepathy> m_lMonitor, m_rMonitor;
  bool m_grabHackEnabled;

  GrabbingScene(bool telepathy=false);

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
  void driveTo(btTransform tf /*meters*/) {
    pr2m->pr2->robot->SetTransform(util::toRaveTransform(tf));
    pr2m->pr2->updateBullet();
  }
  void setFakeHandPoses(btTransform leftPose, btTransform rightPose) {
    m_lMonitor->m_telePose = leftPose;
    m_rMonitor->m_telePose = rightPose;
  }
};

std::vector<btVector3> operator*(const std::vector<btVector3>& in, float a);
btTransform operator*(const btTransform& in, float a);

struct TableRopeScene : public GrabbingScene {
  CapsuleRope::Ptr m_rope;
  BulletObject::Ptr m_table;
  
  TableRopeScene(fs::path ropeFile, bool telepathy=false);
  
};



