#include "rope.h"
#include "ropescene.h"
#include "unistd.h"
#include "util.h"
#include "grabbing.h"
#include "config_bullet.h"
#include "config_viewer.h"


using boost::shared_ptr;
using namespace util;


class CustomKeyHandler : public osgGA::GUIEventHandler {
    RopeScene &scene;
public:
    CustomKeyHandler(RopeScene &scene_) : scene(scene_) { }
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
};

bool CustomKeyHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&) {
    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case 'a':
            printf("got a\n");
            //scene.leftAction->reset();
            //scene.leftAction->toggleAction();
            //scene.runAction(scene.leftAction, BulletConfig::dt);
            break;
        case 's':
            printf("got s\n");
            //scene.rightAction->reset();
            //scene.rightAction->toggleAction();
            //scene.runAction(scene.rightAction, BulletConfig::dt);
            break;
        case ' ':
            //scene.printInfo();
            break;
        }
        break;
    }
    return false;
}


int main(int argc, char *argv[]) {

  RopeSceneConfig::enableIK = RopeSceneConfig::enableHaptics = false;
  RopeSceneConfig::enableRobot = false;
  GeneralConfig::scale = 1.0;

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(RopeSceneConfig());
  parser.read(argc, argv);



  const float table_height = .765;
  const float rope_radius = .01;
  const float segment_len = .025;
  const float table_thickness = .10;
  int nLinks = 50;

  vector<btVector3> ctrlPts;
  for (int i=0; i< nLinks; i++) {
    ctrlPts.push_back(btVector3(.5+segment_len*i,0,table_height+5*rope_radius));
  }


  shared_ptr <btDefaultMotionState> ms(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(-10,0,table_height-table_thickness/2))));
  shared_ptr<BulletObject> table(new BoxObject(0,btVector3(.75,.75,table_thickness/2),ms));

  shared_ptr <btDefaultMotionState> ms2(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(1.5,0.25,1))));
  shared_ptr<BulletObject> cylinder(new CylinderStaticObject(0,0.05,1,ms2));

 // CapsuleRope(const vector<btVector3>& ctrlPoints, float radius_, float angStiffness=.1, float angDamping=1, float linDamping=.75, float angLimit=.4);
  shared_ptr<CapsuleRope> ropePtr(new CapsuleRope(ctrlPts,.01,.1,1,.75,.4));

  RopeScene s;
  s.viewer.addEventHandler(new CustomKeyHandler(s));
  s.env->bullet->setGravity(btVector3(0,0,-10.));
  
  s.env->add(ropePtr);
  s.env->add(table);
  s.env->add(cylinder);
  table->setColor(0,1,0,1);

  vector<BulletObject::Ptr> children =  ropePtr->getChildren();
  for (int j=0; j<children.size(); j++) {
    children[j]->setColor(1,0,0,1);
  }


  vector< vector<double> > joints;
  vector< int > inds;
  read_1d_array(inds, "../data/inds.txt");
  read_2d_array(joints,"../data/vals.txt");

  int step = 0;

  //  btVector3 v = ropePtr->bodies[0]->getCenterOfMassPosition();
  //RobotBase::ManipulatorPtr rarm(s.pr2->robot->GetManipulators()[5]);
  //RobotBase::ManipulatorPtr larm(s.pr2->robot->GetManipulators()[7]);

  btVector3 pos = ropePtr->bodies[0]->getCenterOfMassPosition();
  Grab g(ropePtr->bodies[0].get(),pos,s.env->bullet->dynamicsWorld);
  s.grab_left = g;
  s.grab_left.updatePosition(ropePtr->bodies[0]->getCenterOfMassPosition() + btVector3(0.5,0,0.5));

  btVector3 pos2 = ropePtr->bodies[ropePtr->bodies.size()-1]->getCenterOfMassPosition();
  Grab g2(ropePtr->bodies[ropePtr->bodies.size()-1].get(),pos2,s.env->bullet->dynamicsWorld);
  s.grab_right = g2;
  s.grab_right.updatePosition(ropePtr->bodies[ropePtr->bodies.size()-1]->getCenterOfMassPosition() + btVector3(0,0,0.5));

  s.startViewer();
  s.startFixedTimestepLoop(BulletConfig::dt);
/*
  for (int i=0; i < joints.size() && !s.viewer.done(); i++) {
    pos += btVector3(0,0,.01);
    g.updatePosition(pos);
    cout << i << endl;



    s.step(.01,300,.001);
    usleep(10*1000);
  }
*/
/*
  while(1)
  {
    s.step(.01,300,.001);
    usleep(10*1000);
  }
*/
}
