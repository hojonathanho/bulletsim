#include "knots.h"
#include "rope_scenes.h"
#include "perception/make_bodies.h"
#include "robots/ros2rave.h"
#include <boost/bind.hpp>
#include "robots/pr2.h"
#include "utils/conversions.h"
#include "utils/vector_io.h"

fs::path KNOT_DATA  = fs::path(EXPAND(BULLETSIM_DATA_DIR) "/knots");


void MonitorForGrabbingWithTelekinesis::grab() {
  // grabs nearest object
  cout << "grabbing nearest object" << endl;
  btTransform curPose = m_telekinesis ? m_telePose : m_manip->getTransform();

  int iNear = -1;
  BulletObject::Ptr nearestObj = getNearestBody(m_bodies, curPose.getOrigin(), iNear);
  cout << "grab: " << m_i << endl;

    if (nearestObj->rigidBody->getCenterOfMassPosition().distance(curPose.getOrigin()) < .05*METERS) {
//  if (true) {
    m_grab = new Grab(nearestObj->rigidBody.get(), curPose.getOrigin(), m_world);
    m_i = iNear;
    nearestObj->setColor(0,0,1,1);
  }
}

void MonitorForGrabbingWithTelekinesis::updateGrabPose() {
  if (!m_grab) return;
  btTransform curPose = m_telekinesis ? m_telePose : m_manip->getTransform();
  m_grab->updatePose(curPose);
}


GrabbingScene::GrabbingScene(bool telekinesis)  {
  pr2m.reset(new PR2Manager(*this));
  m_lMonitor.reset(new MonitorForGrabbingWithTelekinesis(pr2m->pr2Left, env->bullet->dynamicsWorld,telekinesis));
  m_rMonitor.reset(new MonitorForGrabbingWithTelekinesis(pr2m->pr2Right, env->bullet->dynamicsWorld, telekinesis));
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
  Scene::VoidCallback cb = boost::bind(&GrabbingScene::drive, this, step, 0);
  addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Left, boost::bind(&GrabbingScene::drive, this, 0, step));
  addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Right, boost::bind(&GrabbingScene::drive, this, 0, -step));
  addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Up, boost::bind(&GrabbingScene::drive, this, -step, 0));
  addVoidKeyCallback(osgGA::GUIEventAdapter::KEY_Down, boost::bind(&GrabbingScene::drive, this, step, 0));

}

void GrabbingScene::step(float dt) {
  m_lMonitor->update();
  m_rMonitor->update();

  Scene::step(dt);
}

TableRopeScene::TableRopeScene(fs::path ropeFile, bool telekinesis) : GrabbingScene(telekinesis) {
  vector<double> firstJoints = doubleVecFromFile((KNOT_DATA / "init_joints_train.txt").string());
  ValuesInds vi = getValuesInds(firstJoints);
  pr2m->pr2->setDOFValues(vi.second, vi.first);
  
  vector<btVector3> tableCornersWorld = toBulletVectors(floatMatFromFile((KNOT_DATA / "table_corners.txt").string())) * METERS;
  vector<btVector3> controlPointsWorld = toBulletVectors(floatMatFromFile(ropeFile.string())) * METERS;

  PlotPoints::Ptr corners(new PlotPoints(20));
  corners->setPoints(tableCornersWorld);
  env->add(corners);

  m_table = makeTable(tableCornersWorld, .1*GeneralConfig::scale);
  float seglen = controlPointsWorld[0].distance(controlPointsWorld[1]);

//  m_rope.reset(new CapsuleRope(controlPointsWorld, fmin(seglen/4.1,.0075*METERS)));
  m_rope.reset(new CapsuleRope(controlPointsWorld, .0075*METERS));
  env->add(m_rope);
  env->add(m_table);
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
