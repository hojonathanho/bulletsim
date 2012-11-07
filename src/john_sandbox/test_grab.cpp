#include "simulation/simplescene.h"
#include "robots/pr2.h"
#include "simulation/bullet_io.h"
int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(BulletConfig());
  parser.addGroup(GeneralConfig());
  parser.read(argc, argv);
  Scene scene;
  PR2Manager pr2m(scene);
  Load(scene.env, scene.rave, EXPAND(BULLETSIM_DATA_DIR) "/xml/box.xml");
  KinBody::LinkPtr grabberLink = pr2m.pr2Right->manip->GetEndEffector();
  RaveObject::Ptr body = getObjectByName(scene.env, scene.rave,"box");
  cout << body->children[0]->rigidBody->getInvInertiaTensorWorld() << endl;
  body->body->SetTransform(util::toRaveTransform(btTransform(btQuaternion::getIdentity(), btVector3(0,0,4))));
  body->updateBullet();
  scene.addVoidKeyCallback('g',boost::bind(&RaveRobotObject::grab, pr2m.pr2.get(),body, grabberLink),"grab");
  scene.addVoidKeyCallback('r',boost::bind(&RaveRobotObject::release, pr2m.pr2.get(),body),"release");
    scene.startViewer();
  scene.startFixedTimestepLoop(.01);
}
