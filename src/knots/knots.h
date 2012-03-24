#pragma once
#include "simulation/simplescene.h"
#include "robots/grabbing.h"
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include "utils/vector_io.h"
#include "perception/make_bodies.h"
#include "robots/ros2rave.h"
#include "robots/pr2.h"
#include "simulation/rope.h"
#include "utils/conversions.h"

using boost::shared_ptr;
namespace fs = boost::filesystem;

extern fs::path KNOT_DATA;

btTransform unscaleTransform(const btTransform& in);

vector<btVector3> unscaleVectors(const vector<btVector3>& in);

btTransform scaleTransform(const btTransform& in);

vector<btVector3> scaleVectors(const vector<btVector3>& in);

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

struct GrabbingScene : public Scene {
public:
  shared_ptr<PR2Manager> pr2m;
  shared_ptr<MonitorForGrabbing> m_lMonitor;
  shared_ptr<MonitorForGrabbing> m_rMonitor;
  btVector3 m_lPos;
  btVector3 m_rPos;
  bool m_grabHackEnabled;

  GrabbingScene();

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
    m_lPos = leftPose.getOrigin();
    m_rPos = rightPose.getOrigin();    
  }
  
};

vector<btVector3> operator*(const vector<btVector3>& in, float a);

struct TableRopeScene : public GrabbingScene {
  CapsuleRope::Ptr m_rope;
  BulletObject::Ptr m_table;
  
  TableRopeScene(fs::path ropeFile);
  
};



