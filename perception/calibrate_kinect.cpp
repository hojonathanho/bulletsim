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

struct LocalConfig : Config {
  static int frameStep;
  LocalConfig() : Config() {
    params.push_back(new Parameter<int>("frameStep", &frameStep, "number of frames to advance by every iteration"));
  }
};
int LocalConfig::frameStep = 3;

struct CustomSceneConfig : Config {
  static int record;
  CustomSceneConfig() : Config() {
    params.push_back(new Parameter<int>("record", &record, "record every n frames (default 0 means record nothing)"));
  }
};
int CustomSceneConfig::record = 0;

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
  };
  CustomKeyHandler *createKeyHandler() { return new CustomKeyHandler(*this); }
};

bool TransformAdjuster::CustomKeyHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &) {
    switch (ea.getEventType()) {
    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey()) {
        case '3':
            state.moving = true; return true;
        case 'e':
            state.rotating = true; return true;
        case ' ':
            state.paused = !state.paused; return true;
        }
        break;
    case osgGA::GUIEventAdapter::KEYUP:
        switch (ea.getKey()) {
        case '3':
            state.moving = false; return true;
        case 'e':
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
            ta.ct.reset(newTrans);
            return true;
        }
        break;
    }
    return false;
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
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);

  //// comm stuff
  setDataRoot("~/comm/pr2_towel");
  FileSubscriber pcSub("kinect","pcd");
  CloudMessage cloudMsg;
  FileSubscriber towelSub("towel_pts","pcd");
  CloudMessage towelPtsMsg; //first message

  FileSubscriber jointSub("joint_states","txt");
  Retimer<VectorMessage<double> > retimer(&jointSub);

  ////////////// create scene
  CustomScene scene;
  static PlotPoints::Ptr kinectPts(new PlotPoints(2));
  CorrPlots corrPlots;
  static PlotPoints::Ptr towelEstPlot(new PlotPoints(4));
  static PlotPoints::Ptr towelObsPlot(new PlotPoints(4));
  towelObsPlot->setDefaultColor(0,1,0,1);

  vector<double> firstJoints = doubleVecFromFile(filePath("data000000000000.txt", "joint_states").string());
  ValuesInds vi = getValuesInds(firstJoints);
  scene.pr2->setDOFValues(vi.second, vi.first);

  // get kinect transform
  btTransform worldFromKinect = getKinectToWorld(scene.pr2->robot);
  CoordinateTransformer CT(worldFromKinect);
  TransformAdjuster ta(CT, scene);
  TransformAdjuster::CustomKeyHandler *keyHandler = ta.createKeyHandler();
  scene.viewer.addEventHandler(keyHandler);

  /////////////// load table
  vector< vector<float> > vv = floatMatFromFile(onceFile("table_corners.txt").string());
  vector<btVector3> tableCornersCam = toBulletVectors(vv);

  vector<btVector3> tableCornersWorld = CT.toWorldFromCamN(tableCornersCam);
  BulletObject::Ptr table = makeTable(tableCornersWorld, .1*METERS);
  table->setColor(0,0,1,.25);

  //////////////// load towel

  towelPtsMsg.fromFiles(towelSub.m_names.getCur());
  vector<btVector3> towelPtsCam = toBulletVectors(towelPtsMsg.m_data);
  vector<btVector3> towelPtsWorld = CT.toWorldFromCamN(towelPtsCam);
  vector<btVector3> towelCorners = toBulletVectors(getCorners(toEigenVectors(towelPtsWorld)));
  BOOST_FOREACH(btVector3& pt, towelCorners) pt += btVector3(.01*METERS,0,0);

  /// add stuff to scene
  MotionStatePtr ms(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,2))));

  scene.env->add(table);
  scene.env->add(kinectPts);
//  scene.env->add(towelEstPlot);
//  scene.env->add(towelObsPlot);

  scene.startViewer();

  while (!scene.viewer.done()) {
    if (!keyHandler->state.paused) {
      for (int z = 0; z < LocalConfig::frameStep - 1; ++z) {
        pcSub.skip();
//        towelSub.skip();
      }
      if (!pcSub.recv(cloudMsg)) break;
//      if (!towelSub.recv(towelPtsMsg)) break;
    }

    ColorCloudPtr cloudWorld(new ColorCloud());
    pcl::transformPointCloud(*cloudMsg.m_data, *cloudWorld, CT.worldFromCamEigen);
    kinectPts->setPoints(cloudWorld);

 //   vector<btVector3> towelObsPts =  CT.toWorldFromCamN(toBulletVectors(towelPtsMsg.m_data));
 //   towelObsPlot->setPoints(towelObsPts);

/*    VectorMessage<double>* jointMsgPtr = retimer.msgAt(cloudMsg.getTime());
    vector<double> currentJoints = jointMsgPtr->m_data;
    ValuesInds vi = getValuesInds(currentJoints);
    scene.pr2->setDOFValues(vi.second, vi.first);*/

    scene.idleFor(0.01);
  }

  return 0;
}
