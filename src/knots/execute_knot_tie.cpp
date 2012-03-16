#include "simulation/simplescene.h"
#include "robots/grabbing.h"
#include <fstream>
#include <iostream>
#include "simulation/bullet_io.h"
#include "utils/vector_io.h"
#include "utils/conversions.h"
#include "simulation/config_bullet.h"
#include "simulation/rope.h"
#include "knots/grabbing_scene.h"
#include "perception/robot_geometry.h"
#include "perception/utils_perception.h"
#include "perception/make_bodies.h"
#include "robots/ros2rave.h"

using namespace std;
using boost::shared_ptr;


#define KNOT_DATA EXPAND(BULLETSIM_DATA_DIR) "/knots"

int main(int argc, char* argv[]) {
  GeneralConfig::scale = 10;


  GrabbingScene scene;

  vector<double> firstJoints = doubleVecFromFile(KNOT_DATA "/init_joints_train.txt");
  ValuesInds vi = getValuesInds(firstJoints);
  scene.pr2m->pr2->setDOFValues(vi.second, vi.first);

  KinectTrans kinectTrans(scene.pr2m->pr2->robot);
  kinectTrans.calibrate(btTransform(btQuaternion(0.669785, -0.668418, 0.222562, -0.234671), btVector3(0.263565, -0.038203, 1.762524)));
  CoordinateTransformer CT(kinectTrans.getKinectTrans());
  vector<btVector3> tableCornersCam = toBulletVectors(floatMatFromFile(KNOT_DATA "/table_corners_train.txt"));
  vector<btVector3> tableCornersWorld = CT.toWorldFromCamN(tableCornersCam);
  BulletObject::Ptr table = makeTable(tableCornersWorld, .1*GeneralConfig::scale);
  vector<btVector3> ropePtsCam = toBulletVectors(floatMatFromFile(KNOT_DATA "/init_rope_test.txt"));
  //  CapsuleRope::Ptr rope(new CapsuleRope(CT.toWorldFromCamN(ropePtsCam), .0075*METERS));
  CapsuleRope::Ptr rope(new CapsuleRope(ropePtsCam, .0075*METERS));
  scene.env->add(rope);
  scene.env->add(table);
  scene.setGrabBodies(rope->children);

  ifstream poseFile(KNOT_DATA "/poses_warped.txt");

  scene.startViewer();
  scene.setSyncTime(false);
  scene.idle(true);

  PlotAxes::Ptr axesLeft(new PlotAxes());
  PlotAxes::Ptr axesRight(new PlotAxes());

  scene.env->add(axesLeft);
  scene.env->add(axesRight);

  while (poseFile.good()) {
    
    btTransform leftPose, rightPose;
    float leftGrip, rightGrip;

    poseFile >> leftPose
            >> leftGrip
            >> rightPose
            >> rightGrip;
    if (poseFile.eof()) break;

    axesLeft->setup(leftPose, .1*METERS);
    axesRight->setup(rightPose, .1*METERS);

    scene.pr2m->pr2Left->moveByIK(leftPose);
    scene.pr2m->pr2Right->moveByIK(rightPose);
    scene.pr2m->pr2Left->setGripperAngle(leftGrip);
    scene.pr2m->pr2Right->setGripperAngle(rightGrip);
    
    scene.step(DT);

  }
  cout << "done with playback" << endl;
}
