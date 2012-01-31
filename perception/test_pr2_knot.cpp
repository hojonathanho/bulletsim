#include "basicobjects.h"
#include "bullet_io.h"
#include "clouds/comm_cv.h"
#include "clouds/comm_pcl.h"
#include "clouds/utils_cv.h"
#include "clouds/utils_pcl.h"
#include "comm/comm2.h"
#include "config.h"
#include "config_bullet.h"
#include "config_perception.h"
#include "grabbing.h"
#include "make_bodies.h"
#include "rope.h"
#include "simplescene.h"
#include "trackers.h"
#include "utils_perception.h"
#include "vector_io.h"
#include "optimization_forces.h"
#include "visibility.h"
#include "apply_impulses.h"
#include "openrave_joints.h"
#include "robot_geometry.h"
#include "recording.h"

#include <pcl/common/transforms.h>

struct LocalConfig : Config {
  static int frameStep;
  LocalConfig() : Config() {
    params.push_back(new Parameter<int>("frameStep", &frameStep, "number of frames to advance by every iteration"));
  }
};
int LocalConfig::frameStep = 3;

struct CustomScene : public Scene {
  MonitorForGrabbing lMonitor;
  MonitorForGrabbing rMonitor;

  CustomScene() : 
    Scene(), 
    lMonitor(pr2->robot->GetManipulators()[5], env->bullet->dynamicsWorld),
    rMonitor(pr2->robot->GetManipulators()[7], env->bullet->dynamicsWorld) {
  };

  void step(float dt) {
    rMonitor.update();
    lMonitor.update();
    Scene::step(dt);
  }
};

int main(int argc, char *argv[]) {

  // command line options
  GeneralConfig::scale = 10;
  SceneConfig::enableIK = SceneConfig::enableHaptics = false;
  SceneConfig::enableRobot = true;
  
  Parser parser;
  parser.addGroup(TrackingConfig());
  parser.addGroup(RecordingConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SceneConfig());
  parser.read(argc,argv);


  // comm stuff
  initComm();
  FileSubscriber pcSub("kinect","pcd");
  CloudMessage cloudMsg;
  FileSubscriber ropeSub("rope_pts","pcd");
  CloudMessage ropeMsg;
  FileSubscriber labelSub("labels","png");
  ImageMessage labelMsg;
  FileSubscriber endSub("rope_ends","txt");
  VecVecMessage<float> endMsg;
  FileSubscriber jointSub("joint_states","txt");
  Retimer<VectorMessage<double> > retimer(&jointSub);


  // get kinect transform
  CustomScene scene;
  vector<double> firstJoints = doubleVecFromFile(filePath("data000000000000.txt", "joint_states").string());
  ValuesInds vi = getValuesInds(firstJoints);
  scene.pr2->setDOFValues(vi.second, vi.first);

  KinectTrans kinectTrans(scene.pr2->robot);
  kinectTrans.calibrate(btTransform(btQuaternion(0.669785, -0.668418, 0.222562, -0.234671), btVector3(0.263565, -0.038203, 1.762524)));
  CoordinateTransformer CT(kinectTrans.getKinectTrans());


  // load table
  /////////////// load table
  vector<btVector3> tableCornersCam = toBulletVectors(floatMatFromFile(onceFile("table_corners.txt").string()));
  vector<btVector3> tableCornersWorld = CT.toWorldFromCamN(tableCornersCam);
  BulletObject::Ptr table = makeTable(tableCornersWorld, .1*GeneralConfig::scale);
  table->setColor(1,1,1,.25);


  // load rope
  vector<btVector3> ropePtsCam = toBulletVectors(floatMatFromFile(onceFile("init_rope.txt").string()));
  CapsuleRope::Ptr rope(new CapsuleRope(CT.toWorldFromCamN(ropePtsCam), .0075*METERS));

  // plots
  PlotPoints::Ptr kinectPts(new PlotPoints(2));
  CorrPlots corrPlots;

  // setup scene
  scene.env->add(kinectPts);
  scene.env->add(rope);
  scene.env->add(table);
  scene.env->add(corrPlots.m_lines);
  scene.lMonitor.setBodies(rope->children);
  scene.rMonitor.setBodies(rope->children);

  // end tracker
  vector<RigidBodyPtr> rope_ends;
  rope_ends.push_back(rope->bodies[0]);
  rope_ends.push_back(rope->bodies[rope->bodies.size()-1]);
  MultiPointTrackerRigid endTracker(rope_ends,scene.env->bullet->dynamicsWorld);
  TrackerPlotter trackerPlotter(endTracker);
  //scene.env->add(trackerPlotter.m_fakeObjects[0]);
  //scene.env->add(trackerPlotter.m_fakeObjects[1]);

  scene.startViewer();
  scene.setSyncTime(true);
  scene.idle(true);

  ScreenRecorder rec(scene.viewer);


  int count=0;
  while (pcSub.recv(cloudMsg)) {
    ENSURE(ropeSub.recv(ropeMsg));
    vector<btVector3> obsPts = CT.toWorldFromCamN(toBulletVectors(ropeMsg.m_data));
    ENSURE(labelSub.recv(labelMsg));
    cv::Mat labels = toSingleChannel(labelMsg.m_data);
    ENSURE(endSub.recv(endMsg));
    vector<btVector3> newEnds = CT.toWorldFromCamN(toBulletVectors(endMsg.m_data));
    endTracker.update(newEnds);
    trackerPlotter.update();

    ColorCloudPtr cloudCam  = cloudMsg.m_data;
    ColorCloudPtr cloudWorld(new ColorCloud());
    pcl::transformPointCloud(*cloudCam, *cloudWorld, CT.worldFromCamEigen);
    kinectPts->setPoints(cloudWorld);
    cout << "loaded cloud " << count << endl;
    count++;


    VectorMessage<double>* jointMsgPtr = retimer.msgAt(cloudMsg.getTime());
    vector<double> currentJoints = jointMsgPtr->m_data;
    ValuesInds vi = getValuesInds(currentJoints);
    scene.pr2->setDOFValues(vi.second, vi.first);
    CT.reset(kinectTrans.getKinectTrans());


    cv::Mat ropeMask = toSingleChannel(labels) == 1;

    for (int iter=0; iter<TrackingConfig::nIter; iter++) {
      cout << "iteration " << iter << endl;

      vector<btVector3> estPts = rope->getNodes();
      Eigen::MatrixXf ropePtsCam = toEigenMatrix(CT.toCamFromWorldN(estPts));
      vector<float> pVis = calcVisibility(rope->bodies, scene.env->bullet->dynamicsWorld, CT.worldFromCamUnscaled.getOrigin()*METERS, TrackingConfig::sigA*METERS, TrackingConfig::nSamples); 
      colorByVisibility(rope, pVis);
      SparseArray corr = toSparseArray(calcCorrProb(toEigenMatrix(estPts), toEigenMatrix(obsPts), toVectorXf(pVis), TrackingConfig::sigB*METERS, TrackingConfig::outlierParam),TrackingConfig::cutoff);
      corrPlots.update(estPts, obsPts, corr);
      vector<btVector3> impulses = calcImpulsesSimple(estPts, obsPts, corr, TrackingConfig::impulseSize);
      applyImpulses(impulses, rope);
      scene.step(DT);

    }
  if (RecordingConfig::record) rec.snapshot();

  }
}
