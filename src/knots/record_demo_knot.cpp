#include <fstream>
#include <iostream>
#include "simulation/bullet_io.h"
#include "simulation/config_bullet.h"
#include <boost/foreach.hpp>
#include "utils/vector_io.h"
#include "utils/conversions.h"
#include "simulation/rope.h"
#include "knots/knots.h"
#include "perception/utils_perception.h"
#include <boost/format.hpp>
#include "utils_python.h"
#include "utils/yes_or_no.h"
#include "trajectory_library.h"
#include "rope_scenes.h"

using namespace std;
namespace fs = boost::filesystem;


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
  static string dbname;
  static bool telekinesis;
  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("initRope", &initRope, "file specifying initial rope")); 
    params.push_back(new Parameter<string>("dbname", &dbname, "database to save file")); 
    params.push_back(new Parameter<bool>("telekinesis", &telekinesis, "use telekinesis")); 
  }
};
string LocalConfig::initRope = "";
string LocalConfig::dbname = "";
bool LocalConfig::telekinesis = false;


int main(int argc, char* argv[]) {

  setup_python();

  SceneConfig::enableHaptics = false;
  GeneralConfig::scale = 10;
  BulletConfig::gravity.m_floats[2] = -30;
  
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(SceneConfig());
  parser.read(argc, argv);
  if (LocalConfig::initRope == "") throw std::runtime_error("must specify a rope file (--initRope=...)");
  if (LocalConfig::dbname == "") throw std::runtime_error("must specify a db name --dbname=)");

  TableRopeScene scene(KNOT_DATA / LocalConfig::initRope, LocalConfig::telekinesis);

  scene.startViewer();
  scene.setSyncTime(true);
  
  scene.addVoidKeyCallback('R',&toggleRecording);
  scene.addVoidKeyCallback('Q',&setWantsExit);
  scene.addVoidKeyCallback('A',boost::bind(&TableRopeScene::closeLeft, &scene));
  scene.addVoidKeyCallback('S',boost::bind(&TableRopeScene::closeRight, &scene));
  scene.addVoidKeyCallback('a',boost::bind(&TableRopeScene::openLeft, &scene));
  scene.addVoidKeyCallback('s',boost::bind(&TableRopeScene::openRight, &scene));

  vector<RobotAndRopeState> rars;

  cout << "meters " << METERS << " " << (1/METERS) << endl;

  while (!wantsExit) {

    if (LocalConfig::telekinesis) {
      if (scene.pr2m->lEngaged) scene.setFakeLeft(scene.pr2m->hapTrackerLeft->rigidBody->getCenterOfMassTransform());
      if (scene.pr2m->rEngaged) scene.setFakeRight(scene.pr2m->hapTrackerRight->rigidBody->getCenterOfMassTransform());
    }



    if (recording) {
      RobotAndRopeState rar;
      if (LocalConfig::telekinesis) {
	rar.leftPose = scene.pr2m->hapTrackerLeft->rigidBody->getCenterOfMassTransform()*(1/METERS);
	rar.rightPose = scene.pr2m->hapTrackerRight->rigidBody->getCenterOfMassTransform()*(1/METERS);
      }
      else {
	rar.leftPose = scene.pr2m->pr2Left->getTransform()*(1/METERS);
	rar.rightPose = scene.pr2m->pr2Right->getTransform()*(1/METERS);
      }
      rar.leftGrip = scene.pr2m->pr2Left->getGripperAngle();
      rar.leftGrab = scene.m_lMonitor->m_i;
      rar.rightGrip = scene.pr2m->pr2Right->getGripperAngle();
      rar.rightGrab = scene.m_rMonitor->m_i;
      rar.ctrlPts = scene.m_rope->getControlPoints()*(1/METERS);

      if (rars.size() == 0)
    	  rars.push_back(rar);
      else {
    	  RobotAndRopeState last_rar = rars[rars.size()-1];
    	  float leftPosDiff = (last_rar.leftPose.getOrigin() - rar.leftPose.getOrigin()).length();
    	  float rightPosDiff = (last_rar.rightPose.getOrigin() - rar.rightPose.getOrigin()).length();
    	  float leftAngDiff = last_rar.leftPose.getRotation().angle(rar.leftPose.getRotation());
    	  float rightAngDiff = last_rar.leftPose.getRotation().angle(rar.rightPose.getRotation());

    	  if (leftPosDiff > .005*METERS || rightPosDiff > .005*METERS || leftAngDiff > .01 || rightAngDiff > .01)
    		  rars.push_back(rar);
      }

    }


    scene.step(DT);
  }

  bool save = rars.size() > 0 ? yesOrNo("save trajectory to library?") : false;
  if (save) {
    try {
      py::object traj_module = py::import("knot_tying.trajectory_library");
      py::object library = traj_module.attr("TrajectoryLibrary")(LocalConfig::dbname,"write");
      py::object stateObj = toNumpy1(rars);
      library.attr("add_demo")(stateObj);
      library.attr("create_segments")();
    }
    catch(py::error_already_set err) {
      PyErr_Print();
      PyErr_Clear();    
    }
  }
}
