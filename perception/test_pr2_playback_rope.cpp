// playback point clouds along with joint states

#include <string>

#include <pcl/common/transforms.h>

#include "config_tracking.h"
#include "comm/comm2.h"
#include "my_exceptions.h"
#include "comm/comm_pcl.h"
#include "vector_io.h"

using namespace std;

const string DATA_ROOT = "/home/joschu/Data/comm_pr2_knot";
const string PCD_TOPIC = "kinect";

float METERS;


int main(int argc, char *argv[]) {


  // set up parameters
  CFG2->read(argc,argv);
  CFG2->scene.enableIK = CFG2->scene.enableHaptics = false;
  CFG2->scene.enableRobot=true;
  CFG2->scene.scale = 10;
  setConfigData(CFG2);
  METERS = CFG2->scene.scale;

  // create comm stuff
  setDataRoot(DATA_ROOT);
  FileSubscriber pcSub("kinect","pcd");
  FileSubscriber jointSub("joints","txt");
  CloudMessage cloudMsg;
  Retimer<MatrixMessage> retimer(&jointSub);

  // find table
  vector<Vector3f> corners_cam;
  Vector3f normal;
  pcSub.recv(cloudMsg);
  getTable(cloudMsg.m_data,corners_cam,normal);
  Affine3f cam2world = getCam2World(corners_cam,normal,METERS);
  btTransform bt_cam2world = toBTTransform(cam2world);
  btVector3 halfExtents;
  btVector3 origin;
  verts2boxPars(corners_world,halfExtents,origin);
  pcl::transformPointCloud(*cloud,*cloud,cam2world);
  BulletObject::Ptr table(new BoxObject(0,halfExtents,ms));

  // set up scene
  Scene scene;
  PlotPoints::Ptr plot(new PlotPoints());
  plot->setPoints(cloud);
  s.env->add(plot);
  s.startViewer();

  // indices of joint angles
  // see /home/joschu/pr2/rope_manipulation/rope_vision/test/write_joint_stuff_to_files.py
  vector<int> inds = read_1d_array("../data/inds.txt");

  while ( pcSub.recv(cloudMsg) ) {

    VectorMessage<double>* jointMsgPtr = retimer.msgAt(cloudMsg.getTime());
    ColorCloud cloud = matMsgPtr->m_data;
    plot->setPoints(cloud);
    vector<double> currentJoints = jointMsgPtr->m_data;
    s.pr2->setDOFValues(inds,currentJoints);
  }

  return 0;
}
