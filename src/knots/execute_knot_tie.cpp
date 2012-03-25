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
#include "arm_base_traj.h"

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
  setup_python();

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
  scene.idle(true);

  PlotAxes::Ptr axesLeft(new PlotAxes());
  PlotAxes::Ptr axesRight(new PlotAxes());

  osg::ref_ptr<PlotCurve> warpedRopePlot = new PlotCurve(4);

  scene.env->add(axesLeft);
  scene.env->add(axesRight);
  scene.env->osg->root->addChild(warpedRopePlot.get());
  
  ifstream poseFile( (KNOT_DATA / LocalConfig::poses).string().c_str() );
  ifstream warpedRopeFile( (KNOT_DATA / LocalConfig::warpedRopes).string().c_str());

  vector<RobotState> states;
  vector<btTransform> leftPoses;
  
  while (poseFile.good()) {
    
    btTransform leftPose, rightPose;
    float leftGrip, rightGrip;


    RobotState rs;
    poseFile >> rs.leftPose
            >> rs.leftGrip
            >> rs.rightPose
            >> rs.rightGrip;
    if (poseFile.eof()) break;
    states.push_back(rs);
    leftPoses.push_back(rs.leftPose);
  }
  

  vector<btTransform> basePoses;
  vector<double> curJoints;
  scene.pr2m->pr2->robot->GetDOFValues(curJoints);
  try {
    basePoses = findBasePoses1(scene.pr2m->pr2Left->manip, leftPoses);
  }
  catch (...) {
    cout << "got a python exception, exiting" << endl;
    PyErr_Print();
    PyErr_Clear();    
    return 1;
  }  

  
  for (int i=0; i < states.size(); i++) {
    axesLeft->setup(states[i].leftPose*METERS, .1*METERS);
    axesRight->setup(states[i].rightPose*METERS, .1*METERS);

    scene.pr2m->pr2Left->moveByIK(states[i].leftPose*METERS);
    scene.pr2m->pr2Right->moveByIK(states[i].rightPose*METERS);
    scene.pr2m->pr2Left->setGripperAngle(states[i].leftGrip);
    scene.pr2m->pr2Right->setGripperAngle(states[i].rightGrip);
    
    scene.driveTo(basePoses[i]);
    
    vector<btVector3> warpedRopePoints = readRopeLine(warpedRopeFile);
    warpedRopePlot->setPoints(warpedRopePoints);

    scene.step(DT);


  }

}
