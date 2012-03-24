#include "knots.h"

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

GrabbingScene::GrabbingScene() : m_grabHackEnabled(false) {
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

void GrabbingScene::step(float dt) {
  m_lMonitor->update();
  m_rMonitor->update();
  if (m_grabHackEnabled && m_lMonitor->m_grab != NULL) m_lMonitor->m_grab->updatePosition(m_lPos);
  if (m_grabHackEnabled && m_rMonitor->m_grab != NULL) m_rMonitor->m_grab->updatePosition(m_lPos);

  Scene::step(dt);
}

TableRopeScene::TableRopeScene(fs::path ropeFile) : GrabbingScene() {
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

vector<btVector3> operator*(const vector<btVector3>& in, float a) {
  vector<btVector3> out(in.size());
  for (int i=0; i < in.size(); i++) {
    out[i] = in[i] * a;
  }
  return out;
}