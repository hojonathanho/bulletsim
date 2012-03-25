#include "knots.h"
#include "perception/make_bodies.h"
#include "robots/ros2rave.h"


fs::path KNOT_DATA  = fs::path(EXPAND(BULLETSIM_DATA_DIR) "/knots");

void MonitorForGrabbingWithTelepathy::grab() {
  // grabs nearest object
  cout << "grabbing nearest object" << endl;
  btTransform curPose = m_telepathy ? m_telePose : m_manip->getTransform();
  BulletObject::Ptr nearestObj = getNearestBody(m_bodies, curPose.getOrigin(), m_i);
  m_grab = new Grab(nearestObj->rigidBody.get(), curPose.getOrigin(), m_world);
  nearestObj->setColor(0,0,1,1);
}

void MonitorForGrabbingWithTelepathy::updateGrabPose() {
  if (!m_grab) return;
  btTransform curPose = m_telepathy ? m_telePose : m_manip->getTransform();
  m_grab->updatePose(curPose);
}


GrabbingScene::GrabbingScene(bool telepathy)  {
  pr2m.reset(new PR2Manager(*this));
  m_lMonitor.reset(new MonitorForGrabbingWithTelepathy(pr2m->pr2Left, env->bullet->dynamicsWorld,telepathy));
  m_rMonitor.reset(new MonitorForGrabbingWithTelepathy(pr2m->pr2Right, env->bullet->dynamicsWorld, telepathy));
  pr2m->setHapticCb(hapticLeft0Hold, boost::bind(&GrabbingScene::closeLeft, this));
  pr2m->setHapticCb(hapticLeft1Hold, boost::bind(&GrabbingScene::openLeft, this));
  pr2m->setHapticCb(hapticRight0Hold, boost::bind(&GrabbingScene::closeRight, this));
  pr2m->setHapticCb(hapticRight1Hold, boost::bind(&GrabbingScene::openRight, this));
  
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

TableRopeScene::TableRopeScene(fs::path ropeFile, bool telepathy) : GrabbingScene(telepathy) {
  vector<double> firstJoints = doubleVecFromFile((KNOT_DATA / "init_joints_train.txt").string());
  ValuesInds vi = getValuesInds(firstJoints);
  pr2m->pr2->setDOFValues(vi.second, vi.first);
  
  vector<btVector3> tableCornersWorld = toBulletVectors(floatMatFromFile((KNOT_DATA / "table_corners.txt").string())) * METERS;
  vector<btVector3> controlPointsWorld = toBulletVectors(floatMatFromFile(ropeFile.string())) * METERS;

  PlotPoints::Ptr corners(new PlotPoints(20));
  corners->setPoints(tableCornersWorld);
  env->add(corners);

  m_table = makeTable(tableCornersWorld, .1*GeneralConfig::scale);
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
