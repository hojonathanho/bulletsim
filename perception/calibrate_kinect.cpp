#include "bullet_io.h"
#include "bullet_typedefs.h"
#include "clouds/comm_pcl.h"
#include "clouds/geom.h"
#include "comm/comm2.h"
#include "config_bullet.h"
#include "config_perception.h"
#include "get_nodes.h"
#include "make_bodies.h"
#include "simplescene.h"
#include "optimization_forces.h"
#include "softbodies.h"
#include "utils_perception.h"
#include "vector_io.h"
#include "visibility.h"
#include "robot_geometry.h"
#include "openrave_joints.h"
#include "grabbing.h"

#include <pcl/common/transforms.h>
#include <osgViewer/ViewerEventHandlers>


struct IncTransform {
  static btTransform rotX(btTransform t,float s) {
    return t*btTransform(btQuaternion(s,0,0,1),btVector3(0,0,0));
  }
  static btTransform rotY(btTransform t,float s) {
    return t*btTransform(btQuaternion(0,s,0,1),btVector3(0,0,0));
  }
  static btTransform rotZ(btTransform t,float s) {
    return t*btTransform(btQuaternion(0,0,s,1),btVector3(0,0,0));
  }
  static btTransform moveX(btTransform t,float s) {
    return t*btTransform(btQuaternion(0,0,0,1),btVector3(s,0,0));
  }
  static btTransform moveY(btTransform t,float s) {
    return t*btTransform(btQuaternion(0,0,0,1),btVector3(0,s,0));
  }
  static btTransform moveZ(btTransform t,float s) {
    return t*btTransform(btQuaternion(0,0,0,1),btVector3(0,0,s));
  }
};

struct CustomSceneConfig : Config {
  static int record;
  CustomSceneConfig() : Config() {
    params.push_back(new Parameter<int>("record", &record, "record every n frames (default 0 means record nothing)"));
  }
};
int CustomSceneConfig::record = 0;

static void printTrans(const btTransform &t) {
    printf("new kinect transform: ");
    btQuaternion q = t.getRotation();
    btVector3 c = t.getOrigin();
    printf("btTransform(btQuaternion(%f, %f, %f, %f), btVector3(%f, %f, %f))\n",
        q.x(), q.y(), q.z(), q.w(), c.x(), c.y(), c.z());
}

class TransformAdjuster {
private:
  CoordinateTransformer &ct;
  Scene &scene;

public:
  TransformAdjuster(CoordinateTransformer &ct_, Scene &scene_) :
    ct(ct_), scene(scene_) { }

  struct CustomKeyHandler : public osgGA::GUIEventHandler {
    TransformAdjuster &ta;
    struct {
      bool moving, rotating, startDragging, paused;
      float lastX, lastY, dx, dy;
    } state;
    CustomKeyHandler(TransformAdjuster &ta_) : ta(ta_), state() { }
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&);
    void setTrans(const btTransform &t) {
      ta.ct.reset(t);
      printTrans(t);
    }
  };
  CustomKeyHandler *createKeyHandler() { return new CustomKeyHandler(*this); }
};

bool TransformAdjuster::CustomKeyHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &) {

#define KEY_CONTROL_LEFT(key, incfunc, delta)				\
  case key:								\
  if (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL) \
    setTrans(IncTransform::incfunc(ta.ct.worldFromCamUnscaled,delta)); \
  break
#define KEY_CONTROL_RIGHT(key, incfunc, delta)				\
  case key:								\
  if (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL) \
    setTrans(IncTransform::incfunc(ta.ct.worldFromCamUnscaled,delta)); \
  break

    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case '-':
            state.moving = true; return true;
        case '=':
            state.rotating = true; return true;
        case ' ':
            state.paused = !state.paused; return true;

        // keyboard control stuff
        KEY_CONTROL_LEFT('q',rotX,.01);
        KEY_CONTROL_LEFT('Q',rotX,-.01);
        KEY_CONTROL_LEFT('w',rotY,.01);
        KEY_CONTROL_LEFT('W',rotY,-.01);
        KEY_CONTROL_LEFT('e',rotZ,.01);
        KEY_CONTROL_LEFT('E',rotZ,-.01);
        KEY_CONTROL_LEFT('a',moveX,.01);
        KEY_CONTROL_LEFT('A',moveX,-.01);
        KEY_CONTROL_LEFT('s',moveY,.01);
        KEY_CONTROL_LEFT('S',moveY,-.01);
        KEY_CONTROL_LEFT('d',moveZ,.01);
        KEY_CONTROL_LEFT('D',moveZ,-.01);

        KEY_CONTROL_RIGHT('p',rotX,.01);
        KEY_CONTROL_RIGHT('P',rotX,-.01);
        KEY_CONTROL_RIGHT('o',rotY,.01);
        KEY_CONTROL_RIGHT('O',rotY,-.01);
        KEY_CONTROL_RIGHT('i',rotZ,.01);
        KEY_CONTROL_RIGHT('I',rotZ,-.01);
        KEY_CONTROL_RIGHT('l',moveX,.01);
        KEY_CONTROL_RIGHT('L',moveX,-.01);
        KEY_CONTROL_RIGHT('k',moveY,.01);
        KEY_CONTROL_RIGHT('K',moveY,-.01);
        KEY_CONTROL_RIGHT('j',moveZ,.01);
        KEY_CONTROL_RIGHT('J',moveZ,-.01);
        }
        break;
    case osgGA::GUIEventAdapter::KEYUP:
        switch (ea.getKey()) {
        case '-':
            state.moving = false; return true;
        case '=':
            state.rotating = false; return true;
        }
        break;
    case osgGA::GUIEventAdapter::PUSH:
        state.startDragging = true;
        return true;
    case osgGA::GUIEventAdapter::DRAG:
        // drag the active manipulator in the plane of view
        if ((ea.getButtonMask() & ea.LEFT_MOUSE_BUTTON) && (state.moving || state.rotating)) {
            if (state.startDragging) {
                state.dx = state.dy = 0;
            } else {
                state.dx = state.lastX - ea.getXnormalized();
                state.dy = ea.getYnormalized() - state.lastY;
            }
            state.lastX = ea.getXnormalized(); state.lastY = ea.getYnormalized();
            state.startDragging = false;
  
            // get our current view
            osg::Vec3d osgCenter, osgEye, osgUp;
            ta.scene.manip->getTransformation(osgCenter, osgEye, osgUp);
            btVector3 from(util::toBtVector(osgEye));
            btVector3 to(util::toBtVector(osgCenter));
            btVector3 up(util::toBtVector(osgUp)); up.normalize();
  
            // compute basis vectors for the plane of view
            // (the plane normal to the ray from the camera to the center of the scene)
            btVector3 normal = (to - from).normalized();
            btVector3 yVec = (up - (up.dot(normal))*normal).normalized(); //FIXME: is this necessary with osg?
            btVector3 xVec = normal.cross(yVec);
            btVector3 dragVec = SceneConfig::mouseDragScale * (state.dx*xVec + state.dy*yVec);

            btTransform origTrans(ta.ct.worldFromCamUnscaled);
            btTransform newTrans(origTrans);
            if (state.moving)
                newTrans.setOrigin(dragVec + origTrans.getOrigin());
            else if (state.rotating) {
                btVector3 axis = normal.cross(dragVec);
                btScalar angle = dragVec.length();
                btQuaternion rot(axis, angle);
                if (rot.length() > 0.99f && rot.length() < 1.01f)
                    newTrans.setRotation(rot * origTrans.getRotation());
            }
            setTrans(newTrans);
            return true;
        }
        break;
    }
    return false;

#undef KEY_CONTROL_LEFT
#undef KEY_CONTROL_RIGHT
}

struct CustomScene : public Scene {
  osgViewer::ScreenCaptureHandler* captureHandler;
  int framecount;
  int captureNumber;

  CustomScene() {
    // add the screen capture handler
    framecount = 0;
    captureHandler = new osgViewer::ScreenCaptureHandler(new osgViewer::ScreenCaptureHandler::WriteToFile("screenshots/img", "jpg", osgViewer::ScreenCaptureHandler::WriteToFile::SEQUENTIAL_NUMBER));
    viewer.addEventHandler(captureHandler);
  };
  void draw() {
    if (CustomSceneConfig::record && framecount % CustomSceneConfig::record==0) captureHandler->captureNextFrame(viewer);
    framecount++;
    Scene::draw();
  }
};


int main(int argc, char* argv[]) {
  //////////// get command line options
  Parser parser;
  SceneConfig::enableIK = SceneConfig::enableHaptics = false;
  SceneConfig::enableRobot = true;
  SceneConfig::enableRobotCollision = false;
  GeneralConfig::scale = 10;
  parser.addGroup(TrackingConfig());
  parser.addGroup(CustomSceneConfig());
  parser.addGroup(SceneConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);

  //// comm stuff
  initComm();
  FileSubscriber pcSub("kinect","pcd");
  CloudMessage cloudMsg;

  FileSubscriber jointSub("joint_states","txt");
  Retimer<VectorMessage<double> > retimer(&jointSub);

  ////////////// create scene
  CustomScene scene;
  static PlotPoints::Ptr kinectPts(new PlotPoints(2));
  scene.env->add(kinectPts);

  vector<double> firstJoints = doubleVecFromFile(filePath("data000000000000.txt", "joint_states").string());
  ValuesInds vi = getValuesInds(firstJoints);
  scene.pr2->setDOFValues(vi.second, vi.first);

  // get kinect transform
  KinectTrans kinectTrans(scene.pr2->robot);
  kinectTrans.calibrate(getKinectToWorld(scene.pr2->robot));//btTransform(btQuaternion(-0.703407, 0.706030, -0.048280, 0.066401), btVector3(0.348212, -0.047753, 1.611060)));
  CoordinateTransformer CT(kinectTrans.getKinectTrans());

  TransformAdjuster ta(CT, scene);
  TransformAdjuster::CustomKeyHandler *keyHandler = ta.createKeyHandler();
  scene.viewer.addEventHandler(keyHandler);

  scene.startViewer();

  ColorCloudPtr cloudWorld(new ColorCloud());
  while (!scene.viewer.done()) {
    if (!keyHandler->state.paused)
      if (!pcSub.recv(cloudMsg)) break;

    VectorMessage<double>* jointMsgPtr = retimer.msgAt(cloudMsg.getTime());
    ValuesInds vi = getValuesInds(jointMsgPtr->m_data);
    scene.pr2->setDOFValues(vi.second, vi.first);
    pcl::transformPointCloud(*cloudMsg.m_data, *cloudWorld, CT.worldFromCamEigen);
    kinectPts->setPoints(cloudWorld);

    scene.step(0.01);
  }

  return 0;
}
