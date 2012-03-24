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
#include "utils/yes_or_np.h"

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

npFloatVector toNumpyArray(const vector<RobotAndRopeState>& rars) {
  assert(rars.size() > 0);
  nCtrlPts = rars[0].ctrlPts.size();
  int nRows = rars.size();
  int nCols = 18+nCtrlPts*3;
  int dims[2] = {nRows, nCols};
  
  npMatrixf out(dims);
  
  for (int i=0; i < rars.size(); i++) {
    out[i][0] = rars[i].leftPose.getOrigin().x();
    out[i][1] = rars[i].leftPose.getOrigin().y();
    out[i][2] = rars[i].leftPose.getOrigin().z();
    out[i][3] = rars[i].leftPose.getRotation().x();
    out[i][4] = rars[i].leftPose.getRotation().y();
    out[i][5] = rars[i].leftPose.getRotation().z();
    out[i][6] = rars[i].leftPose.getRotation().w();
    out[i][7] = rars[i].leftGrip;
    out[i][8] = rars[i].leftGrab;

    out[i][9] = rars[i].rightPose.getOrigin().x();
    out[i][10] = rars[i].rightPose.getOrigin().y();
    out[i][11] = rars[i].rightPose.getOrigin().z();
    out[i][12] = rars[i].rightPose.getRotation().x();
    out[i][13] = rars[i].rightPose.getRotation().y();
    out[i][14] = rars[i].rightPose.getRotation().z();
    out[i][15] = rars[i].rightPose.getRotation().w();
    out[i][16] = rars[i].rightGrip;
    out[i][17] = rars[i].rightGrab;
    
    for (int i=0; i < nCtrlPts; i++) {
      out[i][18+3*i+0] = rars[i].ctrlPts[i].x();
      out[i][18+3*i+1] = rars[i].ctrlPts[i].y();
      out[i][18+3*i+2] = rars[i].ctrlPts[i].z();
    }
    
  }
  
  return out;
  
}

int main(int argc, char* argv[]) {

  setup_python();

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

  vector<RobotAndRopeState> rars;

  while (!wantsExit) {
    if (recording) {
      RobotAndRopeState rar;
      rar.leftPose = scene.pr2m->pr2Left->getTransform();
      rar.leftGrip = scene.pr2m->pr2Left->getGripperAngle();
      rar.leftGrab = scene.m_lMonitor.m_i;
      rar.rightPose = scene.pr2m->pr2Right->getTransform();
      rar.rightGrip = scene.pr2m->pr2Right->getGripperAngle();
      rar.rightGrab = scene.m_rMonitor.m_i;
      rar.ctrlPts = scene.m_rope->getControlPoints();
      rars.push_back(rar);
    }
    scene.step(DT);
  }

  // bool save = yes_or_no("save trajectory to library?");
  bool save = true; // xxx
  if (save) {
    py::object traj_module = py::import("trajectory_library");
    py::object library = traj_module.attr("TrajectoryLibrary")();
    npMatrixf stateArr = toNumpyArray(rars);
    py::object stateObj = toObject(stateArr.py_ptr());
    library.add_demo(toNumpyArray(rars));
  }
}
  

}
