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
#include "physics_aware.h"
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

void adjustWorldTransparency(float inc) {
  static float a=1;
  a=clipf(a+inc,0,1);
  EnvironmentPtr env = util::getGlobalEnv();
  BOOST_FOREACH(EnvironmentObjectPtr obj, env->objects) {
    if (obj) obj->setColor(1,1,1,a);
  }

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
  PR2Manager pr2m(scene);
  ArmPrinter ap(pr2m.pr2Left, pr2m.pr2Right);
  scene.addVoidKeyCallback('c',boost::bind(&ArmPrinter::printCarts, &ap), "print cart");
  scene.addVoidKeyCallback('j',boost::bind(&ArmPrinter::printJoints, &ap), "print joints");


  pr2 = pr2m.pr2;


  RaveRobotObject::Manipulator::Ptr rarm = pr2m.pr2Right;

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

  if (LocalConfig::interactive) {
    scene.startLoop();
  }
  PlanningProblem prob;
  MatrixXd initTraj = makeTraj(startRight, endRight, 20);
  LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, false, defaultMaxStepMvmt(
      initTraj), SQPConfig::lengthCoef));
//  CollisionCostPtr cc(new CollisionCost(pr2->robot, scene.env->bullet->dynamicsWorld, brs,
//      rarm->manip->GetArmIndices(), SQPConfig::distPen, SQPConfig::collCoefInit));
  JointBoundsPtr jb(new JointBounds(true, false, VectorXd::Constant(7, 1e-4), rarm->manip));
  PushObjectPtr po(new PushObject(cyl, scaleTransform(targCylTrans, 1 / METERS), pr2, scene.env,
      pr2m.pr2Right->manip->GetArmIndices(), 10, 0));
  prob.initialize(initTraj, false);
  prob.addComponent(lcc);
//  prob.addComponent(cc);
  prob.addTrustRegionAdjuster(jb);
  prob.addComponent(po);

  TrajPlotterPtr plotter;
  plotter.reset(new ArmPlotter(rarm, &scene, SQPConfig::plotDecimation));
  prob.addPlotter(plotter);
  prob.testObjectives();
  prob.optimize(10000);
  scene.idle(true);


}
