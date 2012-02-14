#include "simulation/basicobjects.h"
#include "simulation/bullet_io.h"
#include "clouds/comm_cv.h"
#include "clouds/comm_pcl.h"
#include "clouds/utils_cv.h"
#include "clouds/utils_pcl.h"
#include "comm/comm.h"
#include "simulation/config_bullet.h"
#include "config_perception.h"
#include "robots/grabbing.h"
#include "make_bodies.h"
#include "simulation/rope.h"
#include "simulation/simplescene.h"
#include "trackers.h"
#include "utils_perception.h"
#include "utils/vector_io.h"
#include "optimization_forces.h"
#include "visibility.h"
#include "apply_impulses.h"
#include "openrave_joints.h"
#include "robot_geometry.h"
#include "simulation/recording.h"
#include <pcl/common/transforms.h>

vector<double> interpolateBetween(vector<double> a, vector<double> b, float p) {
  vector<double> out;
  for (int i=0; i < a.size(); i++) {
    out.push_back(a[i]*(1-p) + b[i]*p);
  }
  cout << p << ": " << out << endl;
  return out;
}

struct LocalConfig : Config {
  static int frameStep;
  LocalConfig() : Config() {
    params.push_back(new Parameter<int>("frameStep", &frameStep, "number of frames to advance by every iteration"));
  }
};
int LocalConfig::frameStep = 3;

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


  Scene scene;
  PR2Manager pr2m(scene);
  MonitorForGrabbing lMonitor(pr2m.pr2->robot->GetManipulators()[5], scene.env->bullet->dynamicsWorld);
  MonitorForGrabbing rMonitor(pr2m.pr2->robot->GetManipulators()[7], scene.env->bullet->dynamicsWorld);

  vector<double> firstJoints = doubleVecFromFile(filePath("data000000000000.txt", "joint_states").string());
  ValuesInds vi = getValuesInds(firstJoints);
  pr2m.pr2->setDOFValues(vi.second, vi.first);

  // get kinect transform
  KinectTrans kinectTrans(pr2m.pr2->robot);
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
  PointCloudPlot::Ptr kinectPts(new PointCloudPlot(2));
  CorrPlots corrPlots;

  // setup scene
  if (TrackingConfig::showKinect) scene.env->add(kinectPts);
  scene.env->add(rope);
  scene.env->add(table);
  if (TrackingConfig:: showLines) scene.env->add(corrPlots.m_lines);
  lMonitor.setBodies(rope->children);
  rMonitor.setBodies(rope->children);

  // recording
  ScreenRecorder* rec;
  if (RecordingConfig::record != DONT_RECORD){
    rec = new ScreenRecorder(scene.viewer);
  }


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

  vector<double> oldvals, newvals;
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
    kinectPts->setPoints1(cloudWorld);
    cout << "loaded cloud " << count << endl;
    count++;


    VectorMessage<double>* jointMsgPtr = retimer.msgAt(cloudMsg.getTime());
    vector<double> currentJoints = jointMsgPtr->m_data;
    ValuesInds vi = getValuesInds(currentJoints);
    newvals = vi.first;
    if (oldvals.size()==0) {
      cout << "first one" << endl;
      oldvals = newvals;
    }


    CT.reset(kinectTrans.getKinectTrans());


    cv::Mat ropeMask = toSingleChannel(labels) == 1;

    for (int iter=0; iter<TrackingConfig::nIter; iter++) {
      cout << "iteration " << iter << endl;

      pr2m.pr2->setDOFValues(vi.second, interpolateBetween(oldvals, newvals, (iter+0.00001)/TrackingConfig::nIter));

      vector<btVector3> estPts = rope->getNodes();
      Eigen::MatrixXf ropePtsCam = toEigenMatrix(CT.toCamFromWorldN(estPts));
      vector<float> pVis = calcVisibility(rope->bodies, scene.env->bullet->dynamicsWorld, CT.worldFromCamUnscaled.getOrigin()*METERS, TrackingConfig::sigA*METERS, TrackingConfig::nSamples); 
      colorByVisibility(rope, pVis);
      SparseArray corr = toSparseArray(calcCorrProb(toEigenMatrix(estPts), toEigenMatrix(obsPts), toVectorXf(pVis), TrackingConfig::sigB*METERS, TrackingConfig::outlierParam),TrackingConfig::cutoff);
      corrPlots.update(estPts, obsPts, corr);
      vector<btVector3> impulses = calcImpulsesSimple(estPts, obsPts, corr, TrackingConfig::impulseSize);
      applyImpulses(impulses, rope);

      if (RecordingConfig::record == EVERY_ITERATION || 
	  RecordingConfig::record == FINAL_ITERATION && iter==TrackingConfig::nIter-1)
	    rec->snapshot();

      lMonitor.update();
      rMonitor.update();

      scene.step(DT);
    }
    oldvals = newvals;

  }
}
