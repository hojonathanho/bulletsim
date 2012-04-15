#include "knots.h"
#include "utils_python.h"
#include "arm_base_traj.h"
#include "rope_scenes.h"
#include "trajectory_library.h"
#include "simulation/config_bullet.h"
#include "simulation/fake_gripper.h"
bool wantsExit = false;
void setWantsExit() {
  wantsExit = true;
}

struct LocalConfig : Config {
  static string dbname;
  static string initRope;
  static bool telekinesis;
  static bool planBase;
  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("initRope",&initRope,"initial rope"));
    params.push_back(new Parameter<string>("dbname", &dbname, "name of database")); 
    params.push_back(new Parameter<bool>("telekinesis", &telekinesis, "actually move grippers or use telekinesis")); 
    params.push_back(new Parameter<bool>("planBase", &planBase, "plan base positions"));
  }
};

string LocalConfig::dbname = "";
string LocalConfig::initRope = "";
bool LocalConfig::telekinesis=true;
bool LocalConfig::planBase=false;

vector<btVector3> getOrigins(const vector<btTransform>& in) {
  vector<btVector3> out;
  out.reserve(in.size());
  BOOST_FOREACH(const btTransform& tf, in) out.push_back(tf.getOrigin());
  return out;
}

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
    parser.addGroup(BulletConfig());
    parser.read(argc, argv); 
    if (LocalConfig::dbname == "") throw std::runtime_error("must specify dbname");



    TableRopeScene scene(KNOT_DATA / LocalConfig::initRope, LocalConfig::telekinesis);
    scene.addVoidKeyCallback('q',&setWantsExit);

    PlotAxes::Ptr warpedLeftAxes(new PlotAxes());
    PlotAxes::Ptr origLeftAxes(new PlotAxes());
    PlotCurve::Ptr warpedRopePlot = new PlotCurve(4);
    PlotCurve::Ptr origRopePlot = new PlotCurve(4);
    // PlotCurve::Ptr origTrajPlot = new PlotCurve(4);
    PlotCurve::Ptr warpedTrajPlot = new PlotCurve(1);
    warpedRopePlot->m_defaultColor = osg::Vec4(1,0,0,1);
    origRopePlot->m_defaultColor = osg::Vec4f(0,0,1,1);  
    // origTrajPlot->m_defaultColor = osg::Vec4f(1,1,0,1);
    warpedTrajPlot->m_defaultColor = osg::Vec4f(1,1,0,1);

    scene.osg->root->addChild(warpedRopePlot.get());
    scene.osg->root->addChild(warpedTrajPlot.get());
    scene.osg->root->addChild(origRopePlot.get());
    scene.env->add(warpedLeftAxes);
    scene.env->add(origLeftAxes);



    scene.startViewer();
    scene.setSyncTime(false);


    py::object library_mod = py::import("knot_tying.rope_library");
    py::object library = library_mod.attr("RopeTrajectoryLibrary")(LocalConfig::dbname, "read");

    while (true) {


      ///////// Trajectory lookup //////////////

      cout << "looking up a close trajectory" << endl;

      py::object rope = ropeToNumpy1(scene.m_rope->getControlPoints()*(1/METERS));
      py::object result = library.attr("get_closest_and_warp")(rope);

      py::object orig_states = result[0];
      py::object warped_states = result[1];

      vector<RobotAndRopeState> rarsOrig = fromNumpy(orig_states);
      vector<RobotAndRopeState> rarsWarped = fromNumpy(warped_states);
      cout << "n states " << rarsOrig.size() << endl;

      vector<btTransform> leftPoses;
      BOOST_FOREACH(RobotAndRopeState rar, rarsWarped) leftPoses.push_back(rar.leftPose);

      vector<btTransform> basePoses;
      if (LocalConfig::planBase) {
        cout << "finding base positions" << endl;
        vector<double> joints; scene.pr2m->pr2->robot->GetDOFValues(joints);
        basePoses = findBasePoses2(rarsWarped, joints);
      }

      assert(rarsOrig.size() == rarsWarped.size());
      if (LocalConfig::planBase) assert(rarsOrig.size() == basePoses.size());

      /////// execute the trajectory you got
      warpedTrajPlot->setPoints(getOrigins(leftPoses)*METERS);
      cout << "executing warped trajectory" << endl;
      for (int i=0; i < rarsWarped.size(); i++) {
        warpedLeftAxes->setup(rarsWarped[i].leftPose*METERS, .2*METERS);
        origLeftAxes->setup(rarsOrig[i].leftPose*METERS, .1*METERS);
        warpedRopePlot->setPoints(rarsWarped[i].ctrlPts*METERS);
        origRopePlot->setPoints(rarsOrig[i].ctrlPts*METERS);

        if (LocalConfig::planBase)
          scene.driveTo(basePoses[i]);

        if (LocalConfig::telekinesis) 
          scene.setFakeHandPoses(rarsWarped[i].leftPose*METERS, rarsWarped[i].rightPose*METERS);
        else {
          scene.pr2m->pr2Left->moveByIK(rarsWarped[i].leftPose*METERS);
          scene.pr2m->pr2Right->moveByIK(rarsWarped[i].rightPose*METERS);
        }

        scene.pr2m->pr2Left->setGripperAngle(rarsWarped[i].leftGrip);    
        scene.pr2m->pr2Right->setGripperAngle(rarsWarped[i].rightGrip);    
        scene.step(DT);
        if (wantsExit) return 0;    
      } // executing a given trajectory
      cout << "done executing trajectory" << endl;

    } // lookup & execute loop


  } // python try block
  catch (std::exception err) {
    PyErr_Print();
    PyErr_Clear();    
  }  
}
