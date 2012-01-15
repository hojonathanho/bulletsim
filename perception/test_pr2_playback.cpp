// playback point clouds along with joint states

#include <string>

#include "clouds/comm_pcl.h"
#include "comm/comm2.h"
#include "config_bullet.h"
#include "config_perception.h"
#include "make_bodies.h"
#include "simplescene.h"
#include "utils_perception.h"
#include "vector_io.h"

using namespace std;
using namespace Eigen;

const string DATA_ROOT = "/home/joschu/Data/comm3";
const string PCD_TOPIC = "kinect";

int main(int argc, char *argv[]) {
  GeneralConfig::scale = 10;
  SceneConfig::enableIK = SceneConfig::enableHaptics = false;
  SceneConfig::enableRobot = true;


  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(TrackingConfig());
  parser.addGroup(SceneConfig());

  // create comm stuff
  setDataRoot(DATA_ROOT);
  FileSubscriber pcSub("kinect","pcd");
  FileSubscriber jointSub("joints","txt");
  CloudMessage cloudMsg;
  Retimer<VectorMessage<double> > retimer(&jointSub);

  // create scene
  Scene scene;
  PlotPoints::Ptr kinectPts(new PlotPoints(2));
  PlotPoints::Ptr ropeObsPts(new PlotPoints(10));
  
  // load table
  vector< vector<float> > vv = matFromFile<float>(onceFile("table_corners.txt").string());
  vector<btVector3> tableCornersCam = toBulletVectors(vv);
  CoordinateTransformer CT(getCamToWorldFromTable(tableCornersCam));
  vector<btVector3> tableCornersWorld = CT.toWorldFromCamN(tableCornersCam);
  BulletObject::Ptr table = makeTable(tableCornersWorld, .1*METERS);
  table->setColor(1,1,1,.25);


  // add stuff to scene
  scene.env->add(table);
  scene.env->add(ropeObsPts);
  scene.startViewer();
  scene.idle(true);

  // indices of joint angles
  // see /home/joschu/pr2/rope_manipulation/rope_vision/test/write_joint_stuff_to_files.py
  vector<int> inds = vecFromFile<int>(string("../data/inds.txt"));

  while ( pcSub.recv(cloudMsg) ) {

    VectorMessage<double>* jointMsgPtr = retimer.msgAt(cloudMsg.getTime());
    ColorCloud cloud = cloudMsg.m_data;
    kinectPts->setPoints(cloud);
    vector<double> currentJoints = jointMsgPtr->m_data;
    scene.pr2->setDOFValues(inds, currentJoints);
  }

  return 0;
}
