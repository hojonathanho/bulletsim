#include "simulation/basicobjects.h"
#include "bullet_io.h"
#include "clouds/comm_cv.h"
#include "clouds/comm_pcl.h"
#include "clouds/utils_cv.h"
#include "clouds/utils_pcl.h"
#include "comm/comm2.h"
#include "utils/config.h"
#include "simulation/config_bullet.h"
#include "config_perception.h"
#include "make_bodies.h"
#include "simulation/rope.h"
#include "trackers.h"
#include "utils_perception.h"
#include "utils/vector_io.h"
#include "optimization_forces.h"
#include "visibility.h"
#include "apply_impulses.h"
#include "simulation/recording.h"
#include "plotting_perception.h"

#include <pcl/common/transforms.h>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

int main(int argc, char *argv[]) {

  // command line options
  GeneralConfig::scale = 10;
  SceneConfig::enableIK = SceneConfig::enableHaptics = SceneConfig::enableRobot = false;
  
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
  FilePublisher ropePub("human_rope","txt");
  // load table
  /////////////// load table
  vector<btVector3> tableCornersCam = toBulletVectors(floatMatFromFile(onceFile("table_corners.txt").string()));
  CoordinateTransformer CT(getCamToWorldFromTable(tableCornersCam));
  vector<btVector3> tableCornersWorld = CT.toWorldFromCamN(tableCornersCam);
  BulletObject::Ptr table = makeTable(tableCornersWorld, .1*GeneralConfig::scale);
  table->setColor(1,1,1,.25);


  // load rope
  vector<btVector3> ropePtsCam = toBulletVectors(floatMatFromFile(onceFile("init_rope.txt").string()));
  CapsuleRope::Ptr rope(new CapsuleRope(CT.toWorldFromCamN(ropePtsCam), .0075*METERS));

  // plots
  PointCloudPlot::Ptr kinectPts(new PointCloudPlot(2));
  CorrPlots corrPlots;
  PointCloudPlot::Ptr obsPlot(new PointCloudPlot(4));
  obsPlot->setDefaultColor(0,1,0,1);

  // setup scene
  Scene scene;
  if (TrackingConfig::showKinect) scene.env->add(kinectPts);
  scene.env->add(rope);
  scene.env->add(table);
  if (TrackingConfig::showObs) scene.env->add(obsPlot);
  if (TrackingConfig::showLines) scene.env->add(corrPlots.m_lines);


  // recording
  ScreenRecorder* rec;
  if (RecordingConfig::record != DONT_RECORD){
    rec = new ScreenRecorder(scene.viewer);
  }

  // end tracker
  vector<RigidBodyPtr> rope_ends;
  rope_ends.push_back(rope->bodies[0]);
  rope_ends.push_back(rope->bodies[rope->bodies.size()-1]);
  //MultiPointTrackerRigid endTracker(rope_ends,scene.env->bullet->dynamicsWorld);
  //TrackerPlotter trackerPlotter(endTracker);
  //scene.env->add(trackerPlotter.m_fakeObjects[0]);
  //scene.env->add(trackerPlotter.m_fakeObjects[1]);

  scene.startViewer();
  scene.setSyncTime(true);
  if (TrackingConfig::startIdle) scene.idle(true);

  int count=0;
  while (pcSub.recv(cloudMsg)) {
    ColorCloudPtr cloudCam  = cloudMsg.m_data;
    ColorCloudPtr cloudWorld(new ColorCloud());
    if (TrackingConfig::showKinect) {
      pcl::transformPointCloud(*cloudCam, *cloudWorld, CT.worldFromCamEigen);
      kinectPts->setPoints1(cloudWorld);
    }
    cout << "loaded cloud " << count << endl;
    count++;

    ENSURE(ropeSub.recv(ropeMsg));
    vector<btVector3> obsPts = CT.toWorldFromCamN(toBulletVectors(ropeMsg.m_data));
    obsPlot->setPoints(obsPts);
    ENSURE(labelSub.recv(labelMsg));
    cv::Mat labels = toSingleChannel(labelMsg.m_data);
    //ENSURE(endSub.recv(endMsg));
    vector<btVector3> newEnds = CT.toWorldFromCamN(toBulletVectors(endMsg.m_data));
    //endTracker.update(newEnds);
    //trackerPlotter.update();
    Eigen::MatrixXf depthImage = getDepthImage(cloudCam);
    cv::Mat ropeMask = toSingleChannel(labels) == 1;

    for (int iter=0; iter<TrackingConfig::nIter; iter++) {
      cout << "iteration " << iter << endl;
      vector<btVector3> estPts = rope->getNodes();
      Eigen::MatrixXf ropePtsCam = toEigenMatrix(CT.toCamFromWorldN(estPts));
      vector<float> pVis = calcVisibility(ropePtsCam, depthImage, ropeMask); 
      colorByVisibility(rope, pVis);
      SparseArray corr = toSparseArray(calcCorrProb(toEigenMatrix(estPts), toEigenMatrix(obsPts), toVectorXf(pVis), TrackingConfig::sigB, TrackingConfig::outlierParam), TrackingConfig::cutoff);
      corrPlots.update(estPts, obsPts, corr);
      vector<btVector3> impulses = calcImpulsesSimple(estPts, obsPts, corr, TrackingConfig::impulseSize);
      applyImpulses(impulses, rope);
      //usleep(1000*10);
      if (RecordingConfig::record == EVERY_ITERATION || 
	  RecordingConfig::record == FINAL_ITERATION && iter==TrackingConfig::nIter-1)
	rec->snapshot();
      scene.step(DT);
      if (iter==TrackingConfig::nIter-1) {
	vector< vector<float> > vv = toVecVec(estPts);
	ropePub.send(VecVecMessage<float>(vv));
      }
    }

  }
}
