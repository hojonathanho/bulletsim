// playback point clouds along with joint states

#include <string>

#include <pcl/common/transforms.h>
#include "simplescene.h"
#include "comm/comm2.h"
#include "clouds/comm_pcl.h"
#include "my_exceptions.h"
#include "vector_io.h"
#include "config.h"
#include "config_bullet.h"
#include "config_perception.h"
#include "utils_perception.h"
using namespace std;

const string DATA_ROOT = "/home/joschu/Data/comm_pr2_knot";
const string PCD_TOPIC = "kinect";

float METERS;


int main(int argc, char *argv[]) {


  Parser parser;
  parser.addGroup(BulletConfig());
  parser.addGroup(TrackingConfig());
  parser.addGroup(SceneConfig());

  // create comm stuff
  setDataRoot(DATA_ROOT);
  FileSubscriber pcSub("kinect","pcd");
  FileSubscriber jointSub("joints","txt");
  CloudMessage cloudMsg;
  Retimer<VecVecMessage<double> > retimer(&jointSub);

  // create scene
  Scene scene;
  static PlotPoints::Ptr kinectPts(new PlotPoints(2));
  
  // load table
  vector< vector<float> > vv = matFromFile<float>(onceFile("table_corners.txt").string());
  vector<btVector3> tableCornersCam = toBulletVectors(vv);
  CoordinateTransformer CT(getCamToWorldFromTable(tableCornersCam));

  worldFromCamUnscaled = getCamToWorldFromTable(tableCornersCam);
  vector<btVector3> tableCornersWorld = CT.toWorldFromCamN(tableCornersCam);
  BulletObject::Ptr table = makeTable(tableCornersWorld, .1*GeneralConfig::scale);
  table->setColor(1,1,1,.25);


  // add stuff to scene
  scene.env->add(table);
  scene.env->add(ropeObsPts);


  // indices of joint angles
  // see /home/joschu/pr2/rope_manipulation/rope_vision/test/write_joint_stuff_to_files.py
  vector<int> inds = read_1d_array("../data/inds.txt");

  while ( pcSub.recv(cloudMsg) ) {

    VectorMessage<double>* jointMsgPtr = retimer.msgAt(cloudMsg.getTime());
    ColorCloud cloud = matMsgPtr->m_data;
    plot->setPoints(cloud);
    vector<double> currentJoints = jointMsgPtr->m_data;
    scene.pr2->setDOFValues(inds, currentJoints);
  }

  return 0;
}
