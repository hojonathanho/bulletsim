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

using namespace std;

void printOneLine(ostream &stream, const vector<btVector3>& vs) {
  BOOST_FOREACH(btVector3 v, vs) stream << v << " ";
  stream << endl;
}


int main(int argc, char* argv[]) {

  Scene scene;
  scene.startViewer();
  scene.setSyncTime(true);

  PR2Manager pr2m(scene);
  scene.step(0);
  scene.idle(true);

  vector<double> firstJoints = doubleVecFromFile("init_joints.txt");
  vector< btVector3 > ctrlPts = toBulletVectors(floatMatFromFile("init_rope.txt"));
  ValuesInds vi = getValuesInds(firstJoints);
  pr2m.pr2->setDOFValues(vi.second, vi.first);

  ofstream poseFile("joints.txt", std::ios_base::out | std::ios_base::trunc);
  ofstream ropeFile("ropes.txt", std::ios_base::out | std::ios_base::trunc);

CapsuleRope::Ptr rope(new CapsuleRope(ctrlPts, .025*METERS));

  for (int i=0; i < 100; i++) {
    poseFile << pr2m.pr2Left->getTransform() << pr2m.pr2Right->getTransform() << endl;
    printOneLine(ropeFile, rope->getNodes());
    scene.step(DT);
  }

  poseFile.close();
  ropeFile.close();
}
