#include "bullet_io.h"
#include "clouds/comm_pcl.h"
#include "clouds/geom.h"
#include "comm/comm2.h"
#include "simulation/config_bullet.h"
#include "config_perception.h"
#include "make_bodies.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "update_bodies.h"
#include "utils_perception.h"
#include "utils/vector_io.h"
#include "visibility.h"


int main(int argc, char* argv[]) {
  setDataRoot("~/comm/towel");
  //////////// get command line options
  Parser parser;
  SceneConfig::enableRobot = SceneConfig::enableIK = SceneConfig::enableHaptics = false;
  GeneralConfig::scale = 10;
  parser.addGroup(SceneConfig());
  parser.addGroup(TrackingConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);


  ////////////// create scene
  Scene scene;
  static PlotPoints::Ptr kinectPts(new PlotPoints(2));
  static PlotPoints::Ptr towelEstPts(new PlotPoints(10));
  towelEstPts->setDefaultColor(1,0,0,1);
  static PlotPoints::Ptr towelObsPts(new PlotPoints(10));
  towelObsPts->setDefaultColor(0,1,0,1);

  /////////////// load table
  vector< vector<float> > vv = floatMatFromFile(onceFile("table_corners.txt").string());
  vector<btVector3> tableCornersCam = toBulletVectors(vv);
  CoordinateTransformer CT(getCamToWorldFromTable(tableCornersCam));

  vector<btVector3> tableCornersWorld = CT.toWorldFromCamN(tableCornersCam);
  BulletObject::Ptr table = makeTable(tableCornersWorld, .1*GeneralConfig::scale);
  table->setColor(1,1,1,.25);

  //////////////// load towel

  FileSubscriber towelSub("towel_pts","pcd");
  CloudMessage towelPtsMsg0; //first message
  ENSURE(towelSub.recv(towelPtsMsg0));

  CloudMessage towelPtsMsg1; //second mesage
  for (int i=0; i<3; i++) {
    ENSURE(towelSub.recv(towelPtsMsg1));
  }

  vector<btVector3> towelPtsCam = toBulletVectors(towelPtsMsg0.m_data);
  vector<btVector3> towelPtsWorld = CT.toWorldFromCamN(towelPtsCam);
  vector<btVector3> towelCorners = toBulletVectors(getCorners(toEigenVectors(towelPtsWorld)));

  BOOST_FOREACH(btVector3& pt, towelCorners) pt += btVector3(.1*METERS,0,0);
  BulletSoftObject::Ptr towel = makeTowel(towelCorners, scene.env->bullet->softBodyWorldInfo);

  /// add stuff to scene


  scene.env->add(towel);
  scene.env->add(table);
  scene.env->add(kinectPts);
  scene.env->add(towelEstPts);
  scene.env->add(towelObsPts);
  scene.startViewer();

  initTrackingPlots();
  scene.env->add(plots::linesAB);
  scene.env->add(plots::linesBA);

  ///////////// locally optimize towel
  vector<btVector3> newPts =  CT.toWorldFromCamN(toBulletVectors(towelPtsMsg1.m_data));
  towelObsPts->setPoints(newPts);

  for (int i=0; i < TrackingConfig::nIter; i++) {
    cout << i << endl;
    scene.idle(true);
    vector<float> pVis = calcVisibility(towel->softBody.get(), scene.env->bullet->dynamicsWorld, CT.worldFromCamUnscaled.getOrigin()*METERS);
    colorByVisibility(towel->softBody.get(), pVis, towelEstPts);
    vector<btVector3> impulses = clothOptImpulses(towel, newPts);
    applyImpulses(impulses, towel);
    scene.step(.01);
  }

}
