#pragma once
#include "simulation/simplescene.h"
#include "robots/grabbing.h"
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include "utils/vector_io.h"
#include "perception/make_bodies.h"
#include "robots/ros2rave.h"
#include "robots/pr2.h"

using boost::shared_ptr;
namespace fs = boost::filesystem;

fs::path KNOT_DATA  = fs::path(EXPAND(BULLETSIM_DATA_DIR) "/knots");

btTransform unscaleTransform(const btTransform& in) {
  btTransform out;
  out.setOrigin(in.getOrigin() / METERS);
  out.setRotation(in.getRotation());
  return out;
}

vector<btVector3> unscaleVectors(const vector<btVector3>& in) {
  vector<btVector3> out = in;
  BOOST_FOREACH(btVector3& v, out) v /= METERS;
  return out;
}

btTransform scaleTransform(const btTransform& in) {
  btTransform out;
  out.setOrigin(in.getOrigin() * METERS);
  out.setRotation(in.getRotation());
  return out;
}

vector<btVector3> scaleVectors(const vector<btVector3>& in) {
  vector<btVector3> out = in;
  BOOST_FOREACH(btVector3& v, out) v *= METERS;
  return out;
}

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

  GrabbingScene() : m_grabHackEnabled(false) {
    pr2m.reset(new PR2Manager(*this));
    m_lMonitor.reset(new MonitorForGrabbing(pr2m->pr2Left, env->bullet->dynamicsWorld));
    m_rMonitor.reset(new MonitorForGrabbing(pr2m->pr2Right, env->bullet->dynamicsWorld));
    pr2m->setHapticCb(hapticLeft0Hold, boost::bind(&GrabbingScene::closeLeft, this));
    pr2m->setHapticCb(hapticLeft1Hold, boost::bind(&GrabbingScene::openLeft, this));
    pr2m->setHapticCb(hapticRight0Hold, boost::bind(&GrabbingScene::closeRight, this));
    pr2m->setHapticCb(hapticRight1Hold, boost::bind(&GrabbingScene::openRight, this));
    
    float step = .01;
    Scene::VoidCallback cb = boost::bind(&GrabbingScene::drive, this, step, 0);
    addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Left, boost::bind(&GrabbingScene::drive, this, 0, step));
    addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Right, boost::bind(&GrabbingScene::drive, this, 0, -step));
    addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Up, boost::bind(&GrabbingScene::drive, this, step, 0));
    addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Down, boost::bind(&GrabbingScene::drive, this, -step, 0));

  }

  void step(float dt) {
    m_lMonitor->update();
    m_rMonitor->update();
    if (m_grabHackEnabled && m_lMonitor->m_grab != NULL) m_lMonitor->m_grab->updatePosition(m_lPos);
    if (m_grabHackEnabled && m_rMonitor->m_grab != NULL) m_rMonitor->m_grab->updatePosition(m_lPos);

    Scene::step(dt);
  }


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
  

};

vector<btVector3> operator*(const vector<btVector3>& in, float a) {
  vector<btVector3> out(in.size());
  for (int i=0; i < in.size(); i++) {
    out[i] = in[i] * a;
  }
  return out;
}

struct TableRopeScene : public GrabbingScene {
  CapsuleRope::Ptr m_rope;
  BulletObject::Ptr m_table;
  
  TableRopeScene(fs::path ropeFile) : GrabbingScene() {
    vector<double> firstJoints = doubleVecFromFile((KNOT_DATA / "init_joints_train.txt").string());
    ValuesInds vi = getValuesInds(firstJoints);
    pr2m->pr2->setDOFValues(vi.second, vi.first);
    
    vector<btVector3> tableCornersWorld = toBulletVectors(floatMatFromFile((KNOT_DATA / "table_corners.txt").string())) * METERS;
    vector<btVector3> controlPointsWorld = toBulletVectors(floatMatFromFile(ropeFile.string())) * METERS;

    BOOST_FOREACH(btVector3 v, tableCornersWorld) cout << v << " " << endl;

    BOOST_FOREACH(btVector3 v, controlPointsWorld) cout << v << " " << endl;

    PlotPoints::Ptr corners(new PlotPoints(20));
    corners->setPoints(tableCornersWorld);
    env->add(corners);

    m_table = makeTable(tableCornersWorld, .1*GeneralConfig::scale);
    m_rope.reset(new CapsuleRope(controlPointsWorld, .0075*METERS));
    env->add(m_rope);
    env->add(m_table);
    setGrabBodies(m_rope->children);
    
  }
  
};



