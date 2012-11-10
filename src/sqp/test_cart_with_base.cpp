#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/pr2.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "utils/vector_io.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "config_sqp.h"
#include "planning_problems2.h"
#include "state_setter.h"
#include "plotters.h"
#include <osg/Depth>
#include "sqp/kinematics_utils.h"
#include "sqp/traj_sqp.h"
#include "utils_sqp.h"
#include "physics_planning_scene.h"
using namespace std;
using namespace Eigen;
using namespace util;
RaveRobotObject::Ptr pr2;


 #define PPSCENE

int main(int argc, char *argv[]) {


  GeneralConfig::verbose=20000;
  GeneralConfig::scale = 100.;
  BulletConfig::linkPadding = .05;
  BulletConfig::margin = 0;
  SQPConfig::distPen = .05;
  SQPConfig::distDiscSafe = .02;
  SQPConfig::distContSafe = .01;
  SQPConfig::collCoefInit = 1000;
  SQPConfig::padMult=2;
  SQPConfig::nStepsInit = 100;
//  bin/test_cart_with_base --distPen=.06 --linkPadding=.06 --maxSteps=10000 --enablePlot=1 --padMult=2 --margin=0  --scale=100000 --distDiscSafe=.02 --distContSafe=0

  //	BulletConfig::margin = .01;
  Parser parser;
  parser.addGroup(GeneralConfig());
  	parser.addGroup(BulletConfig());

  parser.addGroup(SQPConfig());
  parser.read(argc, argv);

  initializeGRB();

#ifdef PPSCENE
  RaveInstancePtr rave(new RaveInstance());
  rave->env->Load("data/pr2test2.env.xml");
  PhysicsPlanningScene ppscene(rave->env);
  ppscene.startViewer();
  Scene& scene = *ppscene.m_planScene;
#else
  Scene scene;
  Load(scene.env, scene.rave, "data/pr2test2.env.xml");
  setGlobalEnv(scene.env);
  setGlobalScene(&scene);
#endif

  PR2Manager pr2m(scene);
  pr2 = pr2m.pr2;
  RaveRobotObject::Manipulator::Ptr arm = pr2m.pr2Right;
//  removeBodiesFromBullet(pr2->children, scene.env->bullet->dynamicsWorld);

  //	makeFullyTransparent(table);


  btTransform goalTrans(btQuaternion(-0.0454175, 0.988128, 0.112242, 0.0945672), btVector3(3.49486, -1.34015, 1));
  VectorXd startJoints(10);
//  startJoints << -1.832, -0.332, -1.011, -1.437, -1.1, -2.106, 3.074,   -3.4, 1.55, 0;
  startJoints << 0,0,0,0,0,0,0,   -3.4, 1.55, 0;
  setDofVals(pr2m.pr2->robot, pr2m.pr2Right->manip->GetArmIndices(), startJoints.topRows(7), startJoints.bottomRows(3));
  getGlobalScene()->step(0);
  getGlobalScene()->idle(true);

  TrajOptimizer opt;
  setupArmToCartTargetWithBase(opt, goalTrans, arm);



#ifndef PPSCENE
  scene.startViewer();
#endif

  vector<int> dofInds = arm->manip->GetArmIndices();
  dofInds.push_back(pr2->robot->GetJoint("torso_lift_joint")->GetDOFIndex());

  RobotJointSetterPtr robotSetter(new RobotJointSetter(getRobotByName(getGlobalScene()->env, getGlobalScene()->rave, "PR2"), dofInds,true));

  StatePlotterPtr statePlotter(new StatePlotter(robotSetter,getGlobalScene()));

  if (SQPConfig::enablePlot) opt.m_plotters.push_back(statePlotter);


  getGlobalScene()->addVoidKeyCallback('=', boost::bind(&adjustWorldTransparency, .05), "increase opacity");
  getGlobalScene()->addVoidKeyCallback('-', boost::bind(&adjustWorldTransparency, -.05), "decrease opacity");

  trajOuterOpt(opt, AllowedCollisions());

  if (opt.m_plotters.size()) opt.m_plotters[0].reset();

  BulletConfig::linkPadding = 0;
#ifndef PPSCENE
  Scene scene1;
  Load(scene1.env, scene1.rave, "data/pr2test2.env.xml");
  scene1.startViewer();
  setGlobalEnv(scene1.env);
  setGlobalScene(&scene1);
  PR2Manager pr2m1(scene1);
  interactiveTrajPlot(opt.m_traj, pr2m1.pr2.get(), dofInds, &scene1);
#else
  interactiveTrajPlot(opt.m_traj, getRobotByName(ppscene.env, ppscene.rave, "PR2").get(), dofInds, &ppscene);
#endif
}
