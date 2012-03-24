#include "simulation/bullet_io.h"
#include "clouds/comm_pcl.h"
#include "clouds/geom.h"
#include "comm/comm.h"
#include "config_perception.h"
#include "get_nodes.h"
#include "make_bodies.h"
#include "optimization_forces.h"
#include "simulation/bullet_typedefs.h"
#include "simulation/config_bullet.h"
#include "simulation/recording.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "utils/vector_io.h"
#include "utils_perception.h"
#include "visibility.h"
#include <pcl/common/transforms.h>

using namespace Eigen;

int main(int argc, char* argv[]) {
  //////////// get command line options
  Parser parser;
  SceneConfig::enableRobot = SceneConfig::enableIK = SceneConfig::enableHaptics = false;
  GeneralConfig::scale = 10;
  parser.addGroup(TrackingConfig());
  parser.addGroup(RecordingConfig());
  parser.addGroup(SceneConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);

  //// comm stuff
  initComm();
  FileSubscriber pcSub("kinect","pcd");
  CloudMessage cloudMsg;
  FileSubscriber towelSub("towel_pts","pcd");
  CloudMessage towelPtsMsg; //first message
  FilePublisher towelPub("human_towel","txt");



  ////////////// create scene
  Scene scene;
  static PointCloudPlot::Ptr kinectPts(new PointCloudPlot(2));
  CorrPlots corrPlots;
  static PointCloudPlot::Ptr towelEstPlot(new PointCloudPlot(4));
  static PointCloudPlot::Ptr towelObsPlot(new PointCloudPlot(4));
  towelObsPlot->setDefaultColor(0,1,0,1);

  ///////////// recording
  ScreenRecorder* rec;
  if (RecordingConfig::record != DONT_RECORD){
    rec = new ScreenRecorder(scene.viewer);
  }

  /////////////// load table
  vector< vector<float> > vv = floatMatFromFile(onceFile("table_corners.txt").string());
  vector<btVector3> tableCornersCam = toBulletVectors(vv);
  CoordinateTransformer CT(getCamToWorldFromTable(tableCornersCam));

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

  scene.env->add(towel);
  scene.env->add(table);
  if (TrackingConfig::showKinect) scene.env->add(kinectPts);
  if (TrackingConfig::showEst) scene.env->add(towelEstPlot);
  if (TrackingConfig::showObs) scene.env->add(towelObsPlot);
  if (TrackingConfig::showLines) scene.env->add(corrPlots.m_lines);
  scene.startViewer();
  towel->setColor(1,1,0,.5);
  if (TrackingConfig::startIdle) scene.idle(true);

  for (int t=0; ; t++) {
    cout << "time step " << t << endl;
    bool success = pcSub.recv(cloudMsg);

    ENSURE(towelSub.recv(towelPtsMsg));

    ColorCloudPtr cloudCam  = cloudMsg.m_data;
    ColorCloudPtr cloudWorld(new ColorCloud());
    if (TrackingConfig::showKinect) {
      pcl::transformPointCloud(*cloudCam, *cloudWorld, CT.worldFromCamEigen);
      kinectPts->setPoints1(cloudWorld);
    }

    vector<btVector3> towelObsPts =  CT.toWorldFromCamN(toBulletVectors(towelPtsMsg.m_data));
    towelObsPlot->setPoints(towelObsPts);

    for (int iter=0; iter < TrackingConfig::nIter; iter++) {
      cout << "iteration " << iter << endl;
      VectorXf pVis = calcVisibility(towel->softBody.get(), scene.env->bullet->dynamicsWorld, CT.worldFromCamUnscaled.getOrigin()*METERS);
      colorByVisibility(towel->softBody.get(), pVis, towelEstPlot);

      vector<btVector3> towelEstPts = getNodes(towel);
      SparseArray corr = toSparseArray(calcCorrProb(toEigenMatrix(towelEstPts), toEigenMatrix(towelObsPts), pVis, TrackingConfig::sigB, TrackingConfig::outlierParam), TrackingConfig::cutoff);
      corrPlots.update(towelEstPts, towelObsPts, corr);

      vector<btVector3> impulses = calcImpulsesSimple(towelEstPts, towelObsPts, corr, TrackingConfig::impulseSize);
      for (int i=0; i<impulses.size(); i++) towel->softBody->addForce(impulses[i],i);

      if (RecordingConfig::record == EVERY_ITERATION || 
	  RecordingConfig::record == FINAL_ITERATION && iter==TrackingConfig::nIter-1) rec->snapshot();
      scene.step(DT);
      if (iter==TrackingConfig::nIter-1) {
	     vector< vector<float> > vv = toVecVec(towelEstPts);
	      towelPub.send(VecVecMessage<float>(vv));
      }

    }

  }
}
