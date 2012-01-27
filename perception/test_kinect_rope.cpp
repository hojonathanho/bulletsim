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
#include "make_bodies.h"
#include "rope.h"
#include "custom_scene.h"
#include "trackers.h"
#include "utils_perception.h"
#include "vector_io.h"
#include "optimization_forces.h"
#include "visibility.h"
#include "apply_impulses.h"

#include <pcl/common/transforms.h>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>



int main(int argc, char *argv[]) {

  // command line options
  GeneralConfig::scale = 10;
  SceneConfig::enableIK = SceneConfig::enableHaptics = SceneConfig::enableRobot = false;
  
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(TrackingConfig());
  parser.addGroup(SceneConfig());
  parser.addGroup(CustomSceneConfig());
  parser.read(argc,argv);

  // comm stuff
  setDataRoot("~/comm/rope_hands");
  FileSubscriber pcSub("kinect","pcd");
  CloudMessage cloudMsg;
  FileSubscriber ropeSub("rope_pts","pcd");
  CloudMessage ropeMsg;
  FileSubscriber labelSub("labels","png");
  ImageMessage labelMsg;
  FileSubscriber endSub("rope_ends","txt");
  VecVecMessage<float> endMsg;
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
  PlotPoints::Ptr kinectPts(new PlotPoints(2));
  CorrPlots corrPlots;

  // setup scene
  CustomScene scene;
  scene.env->add(kinectPts);
  scene.env->add(rope);
  scene.env->add(table);
  scene.env->add(corrPlots.m_lines);


  // end tracker
  vector<RigidBodyPtr> rope_ends;
  rope_ends.push_back(rope->bodies[0]);
  rope_ends.push_back(rope->bodies[rope->bodies.size()-1]);
  MultiPointTrackerRigid endTracker(rope_ends,scene.env->bullet->dynamicsWorld);
  TrackerPlotter trackerPlotter(endTracker);
  scene.env->add(trackerPlotter.m_fakeObjects[0]);
  scene.env->add(trackerPlotter.m_fakeObjects[1]);

  scene.startViewer();
  scene.setSyncTime(true);
  scene.idle(true);

  int count=0;
  while (pcSub.recv(cloudMsg)) {
    ColorCloudPtr cloudCam  = cloudMsg.m_data;
    ColorCloudPtr cloudWorld(new ColorCloud());
    pcl::transformPointCloud(*cloudCam, *cloudWorld, CT.worldFromCamEigen);
    kinectPts->setPoints(cloudWorld);
    cout << "loaded cloud " << count << endl;
    count++;

    BOOST_VERIFY(ropeSub.recv(ropeMsg));
    vector<btVector3> obsPts = CT.toWorldFromCamN(toBulletVectors(ropeMsg.m_data));
    BOOST_VERIFY(labelSub.recv(labelMsg));
    cv::Mat labels = toSingleChannel(labelMsg.m_data);
    BOOST_VERIFY(endSub.recv(endMsg));
    vector<btVector3> newEnds = CT.toWorldFromCamN(toBulletVectors(endMsg.m_data));
    endTracker.update(newEnds);
    trackerPlotter.update();
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
      scene.step(DT);
      usleep(1000*10);
    }
  }
}
