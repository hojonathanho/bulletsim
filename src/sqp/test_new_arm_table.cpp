#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/pr2.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "utils/vector_io.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "config_sqp.h"
#include "plotters.h"
#include "traj_sqp.h"
#include "kinematics_utils.h"
using namespace std;
using namespace Eigen;
using namespace util;


RaveRobotObject::Ptr pr2;

struct LocalConfig: Config {
  static int startPosture;
  static int endPosture;
  static int plotType;

  LocalConfig() :
    Config() {
    params.push_back(new Parameter<int> ("startPosture", &startPosture, "start posture"));
    params.push_back(new Parameter<int> ("endPosture", &endPosture, "end posture"));
    params.push_back(new Parameter<int> ("plotType", &plotType, "0: grippers, 1: arms"));
  }
};
int LocalConfig::startPosture = 3;
int LocalConfig::endPosture = 1;
int LocalConfig::plotType = 1;

const static double postures[][7] = { { -0.4, 1.0, 0.0, -2.05, 0.0, -0.1, 0.0 }, // 0=untucked
    { 0.062, 1.287, 0.1, -1.554, -3.011, -0.268, 2.988 }, //1=tucked
    { -0.33, -0.35, -2.59, -0.15, -0.59, -1.41, 0.27 }, //2=up
    { -1.832, -0.332, -1.011, -1.437, -1.1, -2.106, 3.074 }, //3=side
    { 0, 0, 0, 0, 0, 0, 0 } }; //4=outstretched



int main(int argc, char *argv[]) {


  BulletConfig::linkPadding = .02;
  BulletConfig::margin = 0;
  SQPConfig::padMult = 1;
  GeneralConfig::verbose=20000;
  GeneralConfig::scale = 10.;

  Parser parser;
  parser.addGroup(GeneralConfig());
  //  parser.addGroup(BulletConfig());
  parser.addGroup(LocalConfig());
  parser.addGroup(SQPConfig());
  parser.read(argc, argv);


  const float table_height = .65;
  const float table_thickness = .06;

  initializeGRB();
  Scene scene;
  util::setGlobalEnv(scene.env);
  util::setGlobalScene(&scene);

  BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale * btVector3(.85, .55, table_thickness
      / 2), btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(1.1, 0,
      table_height - table_thickness / 2))));
  scene.env->add(table);
  PR2Manager pr2m(scene);
  pr2 = pr2m.pr2;
  RaveRobotObject::Manipulator::Ptr rarm = pr2m.pr2Right;
  removeBodiesFromBullet(pr2->children, scene.env->bullet->dynamicsWorld);
  //  BOOST_FOREACH(BulletObjectPtr obj, pr2->children) if(obj) makeFullyTransparent(obj);
  //  makeFullyTransparent(table);


  scene.addVoidKeyCallback('=', boost::bind(&adjustWorldTransparency, .05), "increase opacity");
  scene.addVoidKeyCallback('-', boost::bind(&adjustWorldTransparency, -.05), "decrease opacity");


  int nJoints = 7;
  VectorXd startJoints = Map<const VectorXd> (postures[LocalConfig::startPosture], nJoints);
  rarm->setDOFValues(toDoubleVec(startJoints));
  VectorXd endJoints = Map<const VectorXd> (postures[LocalConfig::endPosture], nJoints);

  scene.startViewer();
  TrajOptimizer trajOpt;
  VectorXd lower,upper;
  VectorXd maxPerIter = VectorXd::Constant(7, .2);
  vector<int> dofInds= pr2m.pr2Right->manip->GetArmIndices();
  getJointLimits(pr2->robot, dofInds, lower, upper);
  cout << lower.size() << " " << upper.size() << " " << maxPerIter.size() << endl;
  trajOpt.setTrustRegion(TrustRegionPtr(new JointBounds(&trajOpt, maxPerIter, lower, upper)));
  trajOpt.addCost(CostPtr(new CollisionCost(&trajOpt, pr2.get(), dofInds, false, SQPConfig::collCoefInit)));
  trajOpt.addCost(CostPtr(new JntLenCost(&trajOpt, .1)));

  trajOpt.m_plotters.push_back(ArmPlotterPtr(new ArmPlotter(pr2m.pr2Right, &scene, 1)));

  trajOpt.initialize(makeTraj(startJoints, endJoints, SQPConfig::nStepsInit), arange(SQPConfig::nStepsInit));
  setStartFixed(trajOpt);
  setEndFixed(trajOpt);

  trajOpt.optimize();
  scene.idle(true);
}
