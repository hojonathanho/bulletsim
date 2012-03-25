#include "knots.h"
#include "utils_python.h"
#include "arm_base_traj.h"
#include "trajectory_library.h"
#include "simulation/config_bullet.h"

struct LocalConfig : Config {
  static string dbname;
  static string initRope;
  static bool telepathy;
  static bool planBase;
  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("initRope",&initRope,"initial rope"));
    params.push_back(new Parameter<string>("dbname", &dbname, "name of database")); 
    params.push_back(new Parameter<bool>("telepathy", &telepathy, "actually move grippers or use telepathy")); 
    params.push_back(new Parameter<bool>("planBase", &planBase, "plan base positions"));
  }
};

string LocalConfig::dbname = "";
string LocalConfig::initRope = "";
bool LocalConfig::telepathy=true;
bool LocalConfig::planBase=false;


int main(int argc, char* argv[]) {
  try {

    // get the name of the closest rope state
    // find warping transformation
    // apply it
    // (maybe) find base positions
    // execute it
    // 
    // to plot: new grippers, new rope, old grippers, old rope, candidate base positions, new hand trajectory, old hand trajectory.
    setup_python();


    GeneralConfig::scale=10;
    Parser parser;
    parser.addGroup(LocalConfig()); 
    parser.addGroup(GeneralConfig());
    parser.read(argc, argv); 
    if (LocalConfig::dbname == "") throw std::runtime_error("must specify dbname");
  
  
  
    TableRopeScene scene(KNOT_DATA / LocalConfig::initRope, LocalConfig::telepathy);
  
    PlotAxes::Ptr warpedLeftAxes(new PlotAxes());
    PlotAxes::Ptr origLeftAxes(new PlotAxes());
    PlotCurve::Ptr warpedRopePlot = new PlotCurve(4);
    PlotCurve::Ptr origRopePlot = new PlotCurve(4);
    warpedRopePlot->m_defaultColor = osg::Vec4(1,0,0,1);
    origRopePlot->m_defaultColor = osg::Vec4f(0,0,1,1);  


    scene.osg->root->addChild(warpedRopePlot.get());
    scene.osg->root->addChild(origRopePlot.get());
    scene.env->add(warpedLeftAxes);
    scene.env->add(origLeftAxes);
  
    py::object rope = ropeToNumpy1(scene.m_rope->getControlPoints()*(1/METERS));
    py::object library_mod = py::import("knot_tying.trajectory_library");
    py::object result = library_mod.attr("get_closest_and_warp")(rope,LocalConfig::dbname);
  
    // already have new gripper, new rope
    py::object orig_states = result[0];
    py::object warped_states = result[1];
  
    vector<RobotAndRopeState> rarsOrig = fromNumpy(orig_states);
    vector<RobotAndRopeState> rarsWarped = fromNumpy(warped_states);
    cout << "n states " << rarsOrig.size() << endl;


    vector<btTransform> basePoses;
    if (LocalConfig::planBase) {
      vector<btTransform> leftPoses;
      BOOST_FOREACH(RobotAndRopeState rar, rarsWarped) leftPoses.push_back(rar.leftPose);
    
      basePoses = findBasePoses1(scene.pr2m->pr2Left->manip, leftPoses);    
    }
  
    assert(rarsOrig.size() == rarsWarped.size());
    if (LocalConfig::planBase) assert(rarsOrig.size() == basePoses.size());
  

    scene.startViewer();
    scene.setSyncTime(true);
  
    for (int i=0; i < rarsWarped.size(); i++) {

      warpedLeftAxes->setup(rarsWarped[i].leftPose*METERS, .2*METERS);
      origLeftAxes->setup(rarsOrig[i].leftPose*METERS, .1*METERS);
      warpedRopePlot->setPoints(rarsWarped[i].ctrlPts*METERS);
      origRopePlot->setPoints(rarsOrig[i].ctrlPts*METERS);
    
      if (LocalConfig::planBase)
	scene.driveTo(basePoses[i]);
        
      if (LocalConfig::telepathy) 
	scene.setFakeHandPoses(rarsWarped[i].leftPose*METERS, rarsWarped[i].rightPose*METERS);
      else
	scene.pr2m->pr2Left->moveByIK(rarsWarped[i].leftPose*METERS);
      
      scene.pr2m->pr2Left->setGripperAngle(rarsWarped[i].leftGrip);    
      scene.step(DT);
    
    }


  }
  catch (py::error_already_set err) {
    PyErr_Print();
    PyErr_Clear();    
  }  
}
