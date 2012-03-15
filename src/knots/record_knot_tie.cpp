#include "simulation/simplescene.h"
#include <opencv2/core/core.hpp>
#include "robots/pr2.h"
#include <fstream>
#include <iostream>
#include "simulation/bullet_io.h"
#include "simulation/config_bullet.h"
#include <boost/foreach.hpp>
#include "utils/vector_io.h"
#include "robots/ros2rave.h"
#include "utils/conversions.h"
#include "simulation/rope.h"
#include "knots/grabbing_scene.h"
#include "perception/robot_geometry.h"
#include "perception/utils_perception.h"
#include "perception/make_bodies.h"

using namespace std;

void printOneLine(ostream &stream, const vector<btVector3>& vs) {
  BOOST_FOREACH(btVector3 v, vs) stream << v << " ";
  stream << endl;
}

#define KNOT_DATA EXPAND(BULLETSIM_DATA_DIR) "/knots"

int main(int argc, char* argv[]) {

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
  vector<btVector3> ropePtsCam = toBulletVectors(floatMatFromFile(KNOT_DATA "/init_rope_train.txt"));
  CapsuleRope::Ptr rope(new CapsuleRope(CT.toWorldFromCamN(ropePtsCam), .0075*METERS));
  scene.env->add(rope);
  scene.env->add(table);
  scene.setGrabBodies(rope->children);

  scene.startViewer();
  scene.setSyncTime(true);
  scene.idle(true);

  ofstream poseFile( KNOT_DATA "/poses_train.txt", std::ios_base::out | std::ios_base::trunc);
  ofstream ropeFile( KNOT_DATA "/ropes_train.txt", std::ios_base::out | std::ios_base::trunc);

  while (true) {
    poseFile << scene.pr2m->pr2Left->getTransform() << " " 
            << scene.pr2m->pr2Left->getGripperAngle() << " "
            << scene.pr2m->pr2Right->getTransform() << " "
            << scene.pr2m->pr2Right->getGripperAngle() << endl;
    printOneLine(ropeFile, rope->getNodes());
    scene.step(DT);
  }

  poseFile.close();
  ropeFile.close();
}
