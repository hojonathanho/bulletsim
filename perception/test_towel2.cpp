#include "config_perception.h"
#include "simplescene.h"
#include "comm/comm2.h"
#include "clouds/comm_pcl.h"
#include "clouds/geom.h"
#include "vector_io.h"
#include "bullet_io.h"
#include "make_bodies.h"
#include "update_bodies.h"
#include "softbodies.h"
#include "utils_perception.h"

#include <algorithm>

btTransform worldFromCamUnscaled;

btVector3 toWorldFromCam(const btVector3& camVec) {
  return GeneralConfig::scale * (worldFromCamUnscaled * camVec);
}

vector<btVector3> toWorldFromCamN(const vector<btVector3>& camVecs) {
  vector<btVector3> worldVecs(camVecs.size());
  transform(camVecs.begin(), camVecs.end(), worldVecs.begin(), toWorldFromCam);
  return worldVecs;
}






int main(int argc, char* argv[]) {
  setDataRoot("/home/joschu/Data/comm_towel2");
  //////////// get command line options
  Parser parser;
  SceneConfig::enableRobot = SceneConfig::enableIK = SceneConfig::enableHaptics = false;
  GeneralConfig::scale = 10;
  parser.addGroup(SceneConfig());
  parser.addGroup(TrackingConfig());
  parser.addGroup(GeneralConfig());
  parser.read(argc, argv);


  ////////////// create scene
  Scene scene;
  static PlotPoints::Ptr kinectPts(new PlotPoints(2));
  static PlotPoints::Ptr towelEstPts(new PlotPoints(5));
  towelEstPts->setDefaultColor(1,0,0,1);
  static PlotPoints::Ptr towelObsPts(new PlotPoints(5));
  towelObsPts->setDefaultColor(0,1,0,1);

  /////////////// load table
  vector< vector<float> > vv = matFromFile<float>(onceFile("table_corners.txt").string());
  vector<btVector3> tableCornersCam = toBulletVectors(vv);
  worldFromCamUnscaled = getCamToWorldFromTable(tableCornersCam);
  vector<btVector3> tableCornersWorld = toWorldFromCamN(tableCornersCam);
  BulletObject::Ptr table = makeTable(tableCornersWorld, .1*GeneralConfig::scale);

  //////////////// load towel

  FileSubscriber towelSub("towel_pts","pcd");
  CloudMessage towelPtsMsg0; //first message
  bool success = towelSub.recv(towelPtsMsg0);
  assert(success);

  CloudMessage towelPtsMsg1; //second mesage
  success = towelSub.recv(towelPtsMsg1);
  assert(success);

  vector<btVector3> towelPtsCam = toBulletVectors(towelPtsMsg0.m_data);
  vector<btVector3> towelPtsWorld = toWorldFromCamN(towelPtsCam);
  vector<btVector3> towelCorners = toBulletVectors(getCorners(toEigenVectors(towelPtsWorld)));
  cout << "towelCorners[0] " << towelCorners[0] << endl;
  cout << "tableCorners[0] " << tableCornersWorld[0] << endl;

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
  vector<btVector3> newPts =  toWorldFromCamN(toBulletVectors(towelPtsMsg1.m_data));
  towelObsPts->setPoints(newPts);


  for (int i=0; i < TrackingConfig::nIter; i++) {
    cout << i << endl;
    scene.idle(true);
    vector<btVector3> curPts = getSoftBodyNodes(towel);
    towelEstPts->setPoints(curPts);
    vector<btVector3> impulses = clothOptImpulses(towel, newPts);
    applyImpulses(impulses, towel);
    scene.step(.01);
  }



}
