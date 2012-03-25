#include "simulation/simplescene.h"
#include "robots/grabbing.h"
#include <fstream>
#include <iostream>
#include "simulation/bullet_io.h"
#include "utils/vector_io.h"
#include "utils/conversions.h"
#include "simulation/config_bullet.h"
#include "simulation/rope.h"
#include "perception/robot_geometry.h"
#include "perception/utils_perception.h"
#include "perception/make_bodies.h"
#include "robots/ros2rave.h"
#include "knots/knots.h"
#include "simulation/plotting.h"

using namespace std;
using boost::shared_ptr;

extern fs::path KNOT_DATA;

vector<btVector3> readRopeLine(istream &stream) {
  string line;							
  vector<btVector3> out;							
  btVector3 v;
  getline(stream, line, '\n');					
  stringstream ss(stringstream::in | stringstream::out); 
  ss << line;							
  while (true) {
    ss >> v;
    if (ss.good()) out.push_back(v);
    else break;
  }
  return out;
}									

struct LocalConfig : Config {
  static string initRope;
  static string poses;
  static string warpedRopes;
  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("initRope", &initRope, "file specifying initial rope")); 
    params.push_back(new Parameter<string>("poses", &poses, "file with poses")); 
    params.push_back(new Parameter<string>("warpedRopes", &warpedRopes, "file with warped ropes of training data"));
  }

};
string LocalConfig::initRope="";
string LocalConfig::poses="";
string LocalConfig::warpedRopes="";

int main(int argc, char* argv[]) {
  GeneralConfig::scale = 10;  
  
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(SceneConfig());
  parser.read(argc, argv);
  if (LocalConfig::initRope == "") throw std::runtime_error("must specify a rope file (--initRope=...)");
  if (LocalConfig::poses == "") throw std::runtime_error("must specify a pose file (--poses=...)");
  if (LocalConfig::warpedRopes == "") throw std::runtime_error("must specify warped ropes file");
  TableRopeScene scene(KNOT_DATA / LocalConfig::initRope);

  scene.startViewer();
  scene.setSyncTime(false);

  scene.m_grabHackEnabled = true;

  PlotAxes::Ptr axesLeft(new PlotAxes());
  PlotAxes::Ptr axesRight(new PlotAxes());

  osg::ref_ptr<PlotCurve> warpedRopePlot = new PlotCurve(4);

  scene.env->add(axesLeft);
  scene.env->add(axesRight);
  scene.env->osg->root->addChild(warpedRopePlot.get());
  
  ifstream poseFile( (KNOT_DATA / LocalConfig::poses).string().c_str() );
  ifstream warpedRopeFile( (KNOT_DATA / LocalConfig::warpedRopes).string().c_str());

  while (poseFile.good()) {
    
    btTransform leftPose, rightPose;
    float leftGrip, rightGrip;

    poseFile >> leftPose
            >> leftGrip
            >> rightPose
            >> rightGrip;
    if (poseFile.eof()) break;

    leftPose = leftPose*METERS;
    rightPose = rightPose*METERS;
    
    scene.setFakeHandPoses(leftPose, rightPose);

    axesLeft->setup(leftPose, .1*METERS);
    axesRight->setup(rightPose, .1*METERS);

    scene.pr2m->pr2Left->moveByIK(leftPose);
    scene.pr2m->pr2Right->moveByIK(rightPose);
    scene.pr2m->pr2Left->setGripperAngle(leftGrip);
    scene.pr2m->pr2Right->setGripperAngle(rightGrip);
    
    vector<btVector3> warpedRopePoints = readRopeLine(warpedRopeFile)*METERS;
    warpedRopePlot->setPoints(warpedRopePoints);

    scene.step(DT);

  }

}
