#include "simulation/bullet_typedefs.h"
#include "simulation/config_bullet.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/recording.h"

#include "config_perception.h"
#include "get_nodes.h"
#include "make_bodies.h"
#include "optimization_forces.h"
#include "visibility.h"
#include "robot_geometry.h"
#include "openrave_joints.h"

#include "utils_perception.h"
#include "utils/vector_io.h"
#include "bullet_io.h"

#include "clouds/comm_pcl.h"
#include "clouds/geom.h"
#include "comm/comm.h"

#include "robots/grabbing.h"
#include "robots/pr2.h"

#include <pcl/common/transforms.h>
#include <osgViewer/ViewerEventHandlers>

struct LocalConfig : Config {
  static int frameStep;
  LocalConfig() : Config() {
    params.push_back(new Parameter<int>("frameStep", &frameStep, "number of frames to advance by every iteration"));
  }
};
int LocalConfig::frameStep = 3;

int main(int argc, char* argv[]) {
  //////////// get command line options
  Parser parser;
  SceneConfig::enableIK = SceneConfig::enableHaptics = false;
  SceneConfig::enableRobot = true;
  SceneConfig::enableRobotCollision = false;
  GeneralConfig::scale = 10;
  parser.addGroup(TrackingConfig());
  parser.addGroup(RecordingConfig());
  parser.addGroup(SceneConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);

  //// comm stuff
  initComm();
  FileSubscriber pcSub("kinect","pcd");
  CloudMessage cloudMsg;
  FileSubscriber towelSub("towel_pts","pcd");
  CloudMessage towelPtsMsg; //first message

  FileSubscriber jointSub("joint_states","txt");
  Retimer<VectorMessage<double> > retimer(&jointSub);

  ////////////// create scene
  Scene scene;
  PR2Manager pr2m(scene);
  SoftMonitorForGrabbing lMonitor(pr2m.pr2, true);
  SoftMonitorForGrabbing rMonitor(pr2m.pr2, false);

  PointCloudPlot::Ptr kinectPts(new PointCloudPlot(2));

  CorrPlots corrPlots;
  static PointCloudPlot::Ptr towelEstPlot(new PointCloudPlot(3));
  static PointCloudPlot::Ptr towelObsPlot(new PointCloudPlot(4));
  towelObsPlot->setDefaultColor(0,1,0,1);

  vector<double> firstJoints = doubleVecFromFile(filePath("data000000000000.txt", "joint_states").string());
  ValuesInds vi = getValuesInds(firstJoints);
  pr2m.pr2->setDOFValues(vi.second, vi.first);

  // get kinect transform
  KinectTrans kinectTrans(pr2m.pr2->robot);
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

  scene.env->add(towel);
  scene.env->add(table);
  if (TrackingConfig::showKinect) scene.env->add(kinectPts);
  if (TrackingConfig::showEst) scene.env->add(towelEstPlot);
  if (TrackingConfig::showObs) scene.env->add(towelObsPlot);
  if (TrackingConfig::showLines) scene.env->add(corrPlots.m_lines);
  lMonitor.setTarget(towel->softBody.get());
  rMonitor.setTarget(towel->softBody.get());

  scene.startViewer();
  towel->setColor(1,1,0,.5);

  // recording
  ScreenRecorder* rec;
  if (RecordingConfig::record != DONT_RECORD){
    rec = new ScreenRecorder(scene.viewer);
  }
  scene.idle(true);


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
    pr2m.pr2->setDOFValues(vi.second, vi.first);
    CT.reset(kinectTrans.getKinectTrans());

    pcl::transformPointCloud(*cloudMsg.m_data, *cloudWorld, CT.worldFromCamEigen);
    kinectPts->setPoints1(cloudWorld);

    vector<btVector3> towelObsPts =  CT.toWorldFromCamN(toBulletVectors(towelPtsMsg.m_data));
    towelObsPlot->setPoints(towelObsPts);

    for (int iter=0; iter < TrackingConfig::nIter; iter++) {
      //cout << "iteration " << i << endl;
      cout << "NUMBER OF ANCHORS: " << towel->softBody->m_anchors.size() << endl;
      vector<btVector3> p;
      for (int z = 0; z < towel->softBody->m_anchors.size(); ++z) {
        btSoftBody::Anchor &a = towel->softBody->m_anchors[z];
        btVector3 &x = a.m_node->m_x;
        cout << "\tanchor at " << x.x() << ' ' << x.y() << ' ' << x.z() << endl;
        p.push_back(btVector3(0, 0, 0));
        p.push_back(x);
      }
      scene.plotLines->setPoints(p);

      vector<float> pVis = calcVisibility(towel->softBody.get(), scene.env->bullet->dynamicsWorld, CT.worldFromCamUnscaled.getOrigin()*METERS);
      colorByVisibility(towel->softBody.get(), pVis, towelEstPlot);

      vector<btVector3> towelEstPts = getNodes(towel);
      SparseArray corr = toSparseArray(calcCorrProb(toEigenMatrix(towelEstPts), toEigenMatrix(towelObsPts), toVectorXf(pVis), TrackingConfig::sigB, TrackingConfig::outlierParam), TrackingConfig::cutoff);
      corrPlots.update(towelEstPts, towelObsPts, corr);

      vector<btVector3> impulses = calcImpulsesSimple(towelEstPts, towelObsPts, corr, TrackingConfig::impulseSize);
      for (int i=0; i<impulses.size(); i++)
        towel->softBody->addForce(impulses[i],i);

      if (RecordingConfig::record == EVERY_ITERATION || 
	  RecordingConfig::record == FINAL_ITERATION && iter==TrackingConfig::nIter-1)
	rec->snapshot();

      rMonitor.update();
      lMonitor.update();
      scene.step(.01);
    }

  }

  return 0;
}
