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
#include <osg/Depth>
#include "physics_aware2.h"
#include "sqp/plotters.h"
#include "sqp/state_setter.h"
#include "sqp/kinematics_utils.h"
using namespace std;
using namespace Eigen;
using namespace util;

struct LocalConfig: Config {
  static bool interactive;

  LocalConfig() :
    Config() {
    params.push_back(new Parameter<bool> ("interactive", &interactive, ""));
  }
};
bool LocalConfig::interactive = false;

RaveRobotObject::Ptr pr2;

float clipf(float x,float lo,float hi) {
  return fmin(fmax(x,lo),hi);
}

int main(int argc, char *argv[]) {


  GeneralConfig::scale = 10.;
  //	BulletConfig::margin = .01;
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SQPConfig());
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);
  BulletConfig::linkPadding = 0;

  const float table_height = .65;
  const float table_thickness = .06;

  if (GeneralConfig::verbose > 0) getGRBEnv()->set(GRB_IntParam_OutputFlag, 0);

  Scene scene;
  util::setGlobalEnv(scene.env);
  util::setGlobalScene(&scene);
  scene.startViewer();

  BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale * btVector3(.85, .55, table_thickness
      / 2), btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(1.1, 0,
      table_height - table_thickness / 2))));



  float s = .1;
  btTransform initCylTrans(btQuaternion::getIdentity(), btVector3(.5, 0, table_height+s)*METERS);
  btTransform targCylTrans = initCylTrans;
  targCylTrans.setOrigin(targCylTrans.getOrigin() + btVector3(.2*METERS, .2*METERS, 0));
  util::drawAxes(initCylTrans, .1*METERS, scene.env);
  util::drawAxes(targCylTrans, .1*METERS, scene.env);
  BulletObject::Ptr cyl(new BulletObject(1, new btCylinderShapeZ(btVector3(s,s,s)*METERS), initCylTrans, false));
  scene.env->add(table);
  scene.env->add(cyl);
//  scene.env->bullet->dynamicsWorld->removeRigidBody(cyl->rigidBody.get());
  PR2Manager pr2m(scene);
  ArmPrinter ap(pr2m.pr2Left, pr2m.pr2Right);
  scene.addVoidKeyCallback('c',boost::bind(&ArmPrinter::printCarts, &ap), "print cart");
  scene.addVoidKeyCallback('j',boost::bind(&ArmPrinter::printJoints, &ap), "print joints");


  pr2 = pr2m.pr2;


  RaveRobotObject::Manipulator::Ptr arm = pr2m.pr2Right;

  scene.addVoidKeyCallback('=', boost::bind(&adjustWorldTransparency, .05), "increase opacity");
  scene.addVoidKeyCallback('-', boost::bind(&adjustWorldTransparency, -.05), "decrease opacity");


  int nJoints = 7;
  VectorXd startLeft(7), startRight(7);
  startLeft << 0.914316, 0.00293702, 0.1, -2.14333, 2.18594, -1.8366, 2.6228;
  startRight << -0.761731, -0.121769, -1.5, -1.62638, 0.224091, -1.29141, -2.84389;
  pr2m.pr2Right->setDOFValues(toDoubleVec(startRight));
  pr2m.pr2Left->setDOFValues(toDoubleVec(startLeft));

  VectorXd endRight(7);
  endRight << -0.426533, -0.0657363, -1.5, -1.3825, 0.140105, -1.20288, -2.83324;


  VectorXd startState(13);
  startState << startRight, toXYZROD(initCylTrans);
  VectorXd endState(13);
  endState << endRight, toXYZROD(targCylTrans);

  if (LocalConfig::interactive) {
    scene.startLoop();
  }

  PlanningProblem prob;

  RobotJointSetterPtr robotSetter(new RobotJointSetter(pr2, arm->manip->GetArmIndices()));
  ObjectPoseSetterPtr objectSetter(new ObjectPoseSetter(cyl->rigidBody.get()));
  ComboStateSetterPtr comboSetter(new ComboStateSetter());
  comboSetter->addSetter(robotSetter);
  comboSetter->addSetter(objectSetter);
  
	StatePlotterPtr statePlotter(new StatePlotter(comboSetter, &scene));
//   prob.addPlotter(statePlotter);
//	GripperAxesPlotterPtr gap0(new GripperAxesPlotter(arm,0, scene.env,.05*METERS));
//	GripperAxesPlotterPtr gap1(new GripperAxesPlotter(arm,0, scene.env, .07*METERS));
//	TrajChangePlotterPtr changePlotter(new TwoTrajPlotters(gap0, gap1));
	
	
//	prob.addChangePlotter(changePlotter);

  MatrixXd initTraj = makeTraj(startState, endState, SQPConfig::nStepsInit);
  LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, false, defaultMaxStepMvmt(
      initTraj)*10000, SQPConfig::lengthCoef));
//  BulletRaveSyncherPtr brs = syncherFromArm(arm);
  JointBoundsPtr jb(new JointBounds(true, false, VectorXd::Constant(13, 1e-2), arm->manip,6));
//  jb->m_useBall=true;
  PushCollisionPtr pc(new PushCollision(pr2, pr2->robot->GetLink("r_gripper_r_finger_link"), cyl->rigidBody.get(), arm->manip->GetArmIndices()));
  PushCollisionPtr pc1(new PushCollision(pr2, pr2->robot->GetLink("r_gripper_l_finger_link"), cyl->rigidBody.get(), arm->manip->GetArmIndices()));


  prob.initialize(initTraj, false);
//  pc->calcViolCost(Eigen::VectorXd::Ones(13)*0, Eigen::VectorXd::Ones(13)*0);
//  pc->calcViolCost(Eigen::VectorXd::Ones(13)*.1, Eigen::VectorXd::Ones(13)*.1);

//exit(0);

  int n = prob.m_trajVars.rows()-1;
  for (int j=7; j < 13; ++j) prob.m_model->addConstr(prob.m_trajVars(n,j) ==initTraj(n,j));

  prob.addComponent(pc);
  prob.addComponent(pc1);
  prob.addComponent(lcc);
  prob.addTrustRegionAdjuster(jb);

//  prob.testObjectives();
  srand(0);
#if 0
  CostFuncEvaluator cfe(pc);
  CostGradEvaluator cge(pc);
  testMatGrad(cfe, cge, prob.m_currentTraj, 1e-4  );

  double scales[5] = {1e-5, 1e-4, 1e-3, 1e-2, 1e-1};
  BOOST_FOREACH(double scale, scales) {
     printf("scale: %.3e. ncc: %.3e\n", scale, testMatGradBox(cfe, prob.m_currentTraj, cge(prob.m_currentTraj), scale, 100, true));
  }

//  exit(0);
#endif


//    prob.testObjectives();
  try {
    prob.optimize(100);
  }
  catch (GRBException e) {
    cout << e.getMessage() << endl;
  }
  statePlotter->plotTraj(prob.m_currentTraj);
  scene.idle(true);


}
