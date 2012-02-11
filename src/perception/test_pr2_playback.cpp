// playback point clouds along with joint states

#include <string>
#include <pcl/common/transforms.h>

#include "clouds/comm_pcl.h"
#include "comm/comm2.h"
#include "simulation/config_bullet.h"
#include "config_perception.h"
#include "make_bodies.h"
#include "simulation/simplescene.h"
#include "utils_perception.h"
#include "utils/vector_io.h"
#include "bullet_io.h"
#include "openrave_joints.h"
#include "robot_geometry.h"
#include "clouds/correction.h"
#include "robots/pr2.h"

using namespace std;
using namespace Eigen;

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
  initComm();
  FileSubscriber pcSub("kinect","pcd");
  FileSubscriber jointSub("joint_states","txt");
  CloudMessage cloudMsg;
  Retimer<VectorMessage<double> > retimer(&jointSub);

  // create scene
  Scene scene;
  PR2Manager pr2m(scene);
  PlotPoints::Ptr kinectPts(new PlotPoints(2));

  // get kinect transform
  vector<double> firstJoints = doubleVecFromFile(filePath("data000000000000.txt", "joint_states").string());
  ValuesInds vi = getValuesInds(firstJoints);
  pr2m.pr2->setDOFValues(vi.second, vi.first);

  btTransform worldFromKinect = getKinectToWorld(pr2m.pr2->robot);
  CoordinateTransformer CT(worldFromKinect);
  cout << "world form kinect:" << worldFromKinect << endl;

  
  // load table
  vector< vector<float> > vv = floatMatFromFile(onceFile("table_corners.txt").string());
   vector<btVector3> tableCornersCam = toBulletVectors(vv);
  vector<btVector3> tableCornersWorld = CT.toWorldFromCamN(tableCornersCam);
  //cout << "table corners world" << endl << tableCornersWorld;
  BulletObject::Ptr table = makeTable(tableCornersWorld, .1*METERS);
  table->setColor(1,1,1,.25);


  // add stuff to scene
  //scene.env->add(table);
  scene.env->add(kinectPts);
  scene.startViewer();
  scene.setSyncTime(true);

  scene.idle(true);

  // indices of joint angles
  // see /home/joschu/pr2/rope_manipulation/rope_vision/test/write_joint_stuff_to_files.py
  ColorCloudPtr cloudWorld(new pcl::PointCloud<pcl::PointXYZRGB>());

  int count = 0;
  while ( pcSub.recv(cloudMsg) ) {

    cout << "loaded point cloud " << pcSub.m_names.m_id << endl;
    scene.step(.05);
    scene.idle(true);
    VectorMessage<double>* jointMsgPtr = retimer.msgAt(cloudMsg.getTime());
    ColorCloudPtr cloudCam = cloudMsg.m_data;
    transformPointCloud(*cloudCam, *cloudWorld, CT.worldFromCamEigen);
    kinectPts->setPoints(cloudWorld);

    vector<double> currentJoints = jointMsgPtr->m_data;
    ValuesInds vi = getValuesInds(currentJoints);
    pr2m.pr2->setDOFValues(vi.second, vi.first);

  }

  return 0;
}
