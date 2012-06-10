#include "simulation/simplescene.h"
#include "robots/pr2.h"

typedef RaveRobotObject::Manipulator::Ptr ManipPtr;

btTransform rotX(btTransform t,float s) {
  return t*btTransform(btQuaternion(s,0,0,1),btVector3(0,0,0));
}
btTransform rotY(btTransform t,float s) {
  return t*btTransform(btQuaternion(0,s,0,1),btVector3(0,0,0));
}
btTransform rotZ(btTransform t,float s) {
  return t*btTransform(btQuaternion(0,0,s,1),btVector3(0,0,0));
}
btTransform moveX(btTransform t,float s) {
  return btTransform(btQuaternion(0,0,0,1),btVector3(s,0,0))*t;
}
btTransform moveY(btTransform t,float s) {
  return btTransform(btQuaternion(0,0,0,1),btVector3(0,s,0))*t;
}
btTransform moveZ(btTransform t,float s) {
  return btTransform(btQuaternion(0,0,0,1),btVector3(0,0,s))*t;
}

enum Side {
  LEFT,
  RIGHT
};

class GripperMover : public osgGA::GUIEventHandler {
  ManipPtr m_manip;
  Side m_side;
public:
  GripperMover(ManipPtr manip, Side side) : m_manip(manip), m_side(side) {}
  bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) {
    char key = ea.getKey();
    if (key == 0) return false;
    
    if (m_side == LEFT) {

      float inc = ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_LEFT_ALT ? -.01 : .01;

      switch(key) {
      case 'q': m_manip->moveByIK(moveX(m_manip->getTransform(), inc)); return true;
      case 'w': m_manip->moveByIK(moveY(m_manip->getTransform(), inc)); return true;
      case 'e': m_manip->moveByIK(moveZ(m_manip->getTransform(), inc)); return true;
      case 'a': m_manip->moveByIK(rotX(m_manip->getTransform(), inc)); return true;
      case 's': m_manip->moveByIK(rotY(m_manip->getTransform(), inc)); return true;
      case 'd': m_manip->moveByIK(rotZ(m_manip->getTransform(), inc)); return true;
      }

    }
    if (m_side == RIGHT) {

      float inc = ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_RIGHT_ALT ? .01 : -.01;

      switch(key) {
      case 'p': m_manip->moveByIK(moveX(m_manip->getTransform(), inc)); return true;
      case 'o': m_manip->moveByIK(moveY(m_manip->getTransform(), inc)); return true;
      case 'i': m_manip->moveByIK(moveZ(m_manip->getTransform(), inc)); return true;
      case 'l': m_manip->moveByIK(rotX(m_manip->getTransform(), inc)); return true;
      case 'k': m_manip->moveByIK(rotY(m_manip->getTransform(), inc)); return true;
      case 'j': m_manip->moveByIK(rotZ(m_manip->getTransform(), inc)); return true;
      }
    }


  return false;

  }
};

struct SceneWithKeyboardControl : public Scene {

  PR2Manager m_pr2m;
  GripperMover* m_gmLeft;
  GripperMover* m_gmRight;

  SceneWithKeyboardControl() : m_pr2m(*this) {
    m_gmLeft = new GripperMover(m_pr2m.pr2Left, LEFT);
    m_gmRight = new GripperMover(m_pr2m.pr2Right, RIGHT);
    viewer.addEventHandler(m_gmLeft);
    viewer.addEventHandler(m_gmRight);
    viewer.setCameraManipulator(manip);
  }

  ~SceneWithKeyboardControl() {
    delete m_gmLeft;
    delete m_gmRight;
  }


};

int main(int argc, char *argv[]) {
  // first read the configuration from the user

  // and override config values to what we want
  SceneConfig::enableIK = true;

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(SceneConfig());
  parser.read(argc, argv);


  parser.read(argc, argv);    
    
  // construct the scene
  SceneWithKeyboardControl scene;
  // manipulate the scene or add more objects, if desired

  // start the simulation
  scene.startViewer();
  scene.startLoop();

  return 0;
}
