#include "bullet_io.h"
#include "bullet_typedefs.h"
#include "clouds/comm_pcl.h"
#include "clouds/geom.h"
#include "comm/comm2.h"
#include "config_bullet.h"
#include "config_perception.h"
#include "custom_scene.h"
#include "get_nodes.h"
#include "make_bodies.h"
#include "simplescene.h"
#include "optimization_forces.h"
#include "softbodies.h"
#include "utils_perception.h"
#include "vector_io.h"
#include "visibility.h"

#include <pcl/common/transforms.h>


int main(int argc, char* argv[]) {
  //////////// get command line options
  Parser parser;
  SceneConfig::enableRobot = SceneConfig::enableIK = SceneConfig::enableHaptics = false;
  GeneralConfig::scale = 10;
  parser.addGroup(TrackingConfig());
  parser.addGroup(CustomSceneConfig());
  parser.addGroup(SceneConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);

  //// comm stuff
  setDataRoot("~/comm/towel2");
  FileSubscriber pcSub("kinect","pcd");
  CloudMessage cloudMsg;
  FileSubscriber towelSub("towel_pts","pcd");
  CloudMessage towelPtsMsg; //first message



  ////////////// create scene
  CustomScene scene;
  static PlotPoints::Ptr kinectPts(new PlotPoints(2));
  CorrPlots corrPlots;
  static PlotPoints::Ptr towelEstPlot(new PlotPoints(4));
  static PlotPoints::Ptr towelObsPlot(new PlotPoints(4));
  towelObsPlot->setDefaultColor(0,1,0,1);

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




  /// add stuff to scene
  MotionStatePtr ms(new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,2))));

  scene.env->add(towel);
  scene.env->add(table);
  scene.env->add(kinectPts);
  scene.env->add(towelEstPlot);
  scene.env->add(towelObsPlot);
  //scene.env->add(corrPlots.m_lines);
  //scene.env->add(sphere);

  scene.startViewer();
  towel->setColor(1,1,0,.5);


  for (int t=0; ; t++) {
    cout << "time step " << t << endl;
    bool success = pcSub.recv(cloudMsg);
    if (!success) break;
    ASSERT(towelSub.recv(towelPtsMsg));

    ColorCloudPtr cloudCam  = cloudMsg.m_data;
    ColorCloudPtr cloudWorld(new ColorCloud());
    pcl::transformPointCloud(*cloudCam, *cloudWorld, CT.worldFromCamEigen);
    kinectPts->setPoints(cloudWorld);

    vector<btVector3> towelObsPts =  CT.toWorldFromCamN(toBulletVectors(towelPtsMsg.m_data));
    towelObsPlot->setPoints(towelObsPts);

    for (int i=0; i < TrackingConfig::nIter; i++) {
      cout << "iteration " << i << endl;
      //scene.idle(true);
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
}
