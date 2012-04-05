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
#include "knots/knots.h"
#include "perception/robot_geometry.h"
#include "perception/utils_perception.h"
#include "perception/make_bodies.h"
#include <boost/format.hpp>

using namespace std;
namespace fs = boost::filesystem;

void printOneLine(ostream &stream, const vector<btVector3>& vs) {
  BOOST_FOREACH(btVector3 v, vs) stream << v << " ";
  stream << endl;
}

extern fs::path KNOT_DATA;

bool recording = false;
void toggleRecording() {
  recording = !recording;
  cout << "recording " << (recording ? "ON" : "OFF") << endl;
}
bool wantsExit = false;
void setWantsExit() {
  wantsExit = true;
}


struct LocalConfig : Config {
  static string initRope;
  static string prefix;
  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("initRope", &initRope, "file specifying initial rope")); 
    params.push_back(new Parameter<string>("prefix", &prefix, "prefix for saved files")); 
  }
};
string LocalConfig::initRope = "";
string LocalConfig::prefix = "0";


int main(int argc, char* argv[]) {

  SceneConfig::enableHaptics = true;
  GeneralConfig::scale = 10;
  
  
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(SceneConfig());
  parser.read(argc, argv);
  if (LocalConfig::initRope == "") throw std::runtime_error("must specify a rope file (--initRope=...)");

  TableRopeScene scene(KNOT_DATA / LocalConfig::initRope);

  scene.startViewer();
  scene.setSyncTime(true);
  
  scene.addVoidKeyCallback('r',&toggleRecording);
  scene.addVoidKeyCallback('q',&setWantsExit);

  ofstream poseFile( (KNOT_DATA  / (LocalConfig::prefix + ".poses.txt")  ).string().c_str(), std::ios_base::out | std::ios_base::trunc);
  ofstream ropeFile( (KNOT_DATA  / (LocalConfig::prefix + ".ropes.txt")  ).string().c_str(), std::ios_base::out | std::ios_base::trunc);

  while (!wantsExit) {
    if (recording) {
      poseFile << scene.pr2m->pr2Left->getTransform()*(1/METERS) << " " 
              << scene.pr2m->pr2Left->getGripperAngle() << " "
              << scene.pr2m->pr2Right->getTransform()*(1/METERS) << " "
              << scene.pr2m->pr2Right->getGripperAngle() << endl;
      printOneLine(ropeFile, scene.m_rope->getControlPoints()*METERS);
    }
    scene.step(DT);
  }

  poseFile.close();
  ropeFile.close();
}
