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

struct CustomScene : public Scene {
  osgViewer::ScreenCaptureHandler* captureHandler;
  int framecount;
  int captureNumber;
  SoftMonitorForGrabbing lMonitor;
  SoftMonitorForGrabbing rMonitor;

  CustomScene() :
    Scene(),
    lMonitor(pr2, pr2->robot->GetManipulators()[5], true),
    rMonitor(pr2, pr2->robot->GetManipulators()[7], false) {
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

  void step(float dt) {
    rMonitor.update();
    lMonitor.update();
    Scene::step(dt);
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
  KinectTrans kinectTrans(scene.pr2->robot);
  kinectTrans.calibrate(btTransform(btQuaternion(-0.703407, 0.706030, -0.048280, 0.066401), btVector3(0.348212, -0.047753, 1.611060)));
  CoordinateTransformer CT(kinectTrans.getKinectTrans());

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
  BulletSoftObject::Ptr towel = makeSelfCollidingTowel(towelCorners, scene.env->bullet->softBodyWorldInfo);

  /// add stuff to scene
  MotionStatePtr ms(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,2))));

  scene.env->add(towel);
  scene.env->add(table);
  scene.env->add(kinectPts);
  scene.env->add(towelEstPlot);
  scene.env->add(towelObsPlot);
  //scene.env->add(corrPlots.m_lines);
  scene.lMonitor.setTarget(towel->softBody.get());
  scene.rMonitor.setTarget(towel->softBody.get());

  scene.startViewer();
  towel->setColor(1,1,0,.5);



  ColorCloudPtr cloudWorld(new ColorCloud());
  for (int t=0; ; ) {
    cout << "time step " << t << endl;
    for (int z = 0; z < LocalConfig::frameStep - 1; ++z) {
      pcSub.skip();
      towelSub.skip();
    }
    if (!pcSub.recv(cloudMsg)) break;
    if (!towelSub.recv(towelPtsMsg)) break;
    t += LocalConfig::frameStep;

    VectorMessage<double>* jointMsgPtr = retimer.msgAt(cloudMsg.getTime());
    ValuesInds vi = getValuesInds(jointMsgPtr->m_data);
    scene.pr2->setDOFValues(vi.second, vi.first);
    CT.reset(kinectTrans.getKinectTrans());

    pcl::transformPointCloud(*cloudMsg.m_data, *cloudWorld, CT.worldFromCamEigen);
    kinectPts->setPoints(cloudWorld);

    vector<btVector3> towelObsPts =  CT.toWorldFromCamN(toBulletVectors(towelPtsMsg.m_data));
    towelObsPlot->setPoints(towelObsPts);

    for (int i=0; i < TrackingConfig::nIter; i++) {
      cout << "iteration " << i << endl;
      vector<float> pVis = calcVisibility(towel->softBody.get(), scene.env->bullet->dynamicsWorld, CT.worldFromCamUnscaled.getOrigin()*METERS);
      colorByVisibility(towel->softBody.get(), pVis, towelEstPlot);

      vector<btVector3> towelEstPts = getNodes(towel);
      SparseArray corr = toSparseArray(calcCorrProb(toEigenMatrix(towelEstPts), toEigenMatrix(towelObsPts), toVectorXf(pVis), TrackingConfig::sigB, TrackingConfig::outlierParam), TrackingConfig::cutoff);
      corrPlots.update(towelEstPts, towelObsPts, corr);

      vector<btVector3> impulses = calcImpulsesSimple(towelEstPts, towelObsPts, corr, TrackingConfig::impulseSize);
      for (int i=0; i<impulses.size(); i++)
        towel->softBody->addForce(impulses[i],i);

      scene.step(.01);
    }

  }

  return 0;
}
