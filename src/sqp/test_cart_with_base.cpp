#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/pr2.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "utils/vector_io.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "sqp_algorithm.h"
#include "config_sqp.h"
#include "planning_problems.h"
#include "state_setter.h"
#include "plotters.h"
#include <osg/Depth>
using namespace std;
using namespace Eigen;
using namespace util;
RaveRobotObject::Ptr pr2;



int main(int argc, char *argv[]) {


  BulletConfig::linkPadding = .04;
  GeneralConfig::verbose=20000;
  GeneralConfig::scale = 10.;
  SQPConfig::nStepsInit = 100;

  //	BulletConfig::margin = .01;
  Parser parser;
  parser.addGroup(GeneralConfig());
  	parser.addGroup(BulletConfig());

  parser.addGroup(SQPConfig());
  parser.read(argc, argv);


  if (GeneralConfig::verbose > 0) getGRBEnv()->set(GRB_IntParam_OutputFlag, 0);

  Scene scene;
  util::setGlobalEnv(scene.env);
  util::setGlobalScene(&scene);

  Load(scene.env, scene.rave, "data/pr2test2.env.xml");

  PR2Manager pr2m(scene);
  pr2 = pr2m.pr2;
  RaveRobotObject::Manipulator::Ptr arm = pr2m.pr2Right;
  removeBodiesFromBullet(pr2->children, scene.env->bullet->dynamicsWorld);

  //	makeFullyTransparent(table);


  btTransform goalTrans(btQuaternion(-0.0454175, 0.988128, 0.112242, 0.0945672), btVector3(3.49486, -1.34015, 1));
  VectorXd startJoints(10);
//  startJoints << -1.832, -0.332, -1.011, -1.437, -1.1, -2.106, 3.074,   -3.4, 1.55, 0;
  startJoints << 0,0,0,0,0,0,0,   -3.4, 1.55, 0;

//  util::drawAxes(scaleTransform(goalTrans, METERS), .1*METERS, scene.env);
  util::drawSpheres(goalTrans.getOrigin()*METERS, Vector3f(1,0,0), 1, .05*METERS, scene.env);

  PlanningProblem prob;

  scene.startViewer();


  RobotJointSetterPtr robotSetter(new RobotJointSetter(pr2, arm->manip->GetArmIndices(),true));
  StatePlotterPtr statePlotter(new StatePlotter(robotSetter, &scene));

  if (SQPConfig::enablePlot) prob.addPlotter(statePlotter);


  scene.addVoidKeyCallback('=', boost::bind(&adjustWorldTransparency, .05), "increase opacity");
  scene.addVoidKeyCallback('-', boost::bind(&adjustWorldTransparency, -.05), "decrease opacity");


  TIC();
  bool success = planArmBaseToCartTarget(prob, startJoints, goalTrans, arm);
  LOG_INFO("total time: " << TOC());
  if (prob.m_plotters.size()) prob.m_plotters[0].reset();

  BulletConfig::linkPadding = 0;
  scene.env->remove(pr2);
  PR2Manager pr2m1(scene);
  interactiveTrajPlot(prob.m_currentTraj, pr2m1.pr2Right,&scene);

}
