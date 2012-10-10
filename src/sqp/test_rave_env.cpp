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
#include <json/json.h>
#include <boost/filesystem.hpp>
#include "utils/my_exceptions.h"
#include "planning_problems.h"
using namespace std;
using namespace Eigen;
using namespace util;
namespace fs = boost::filesystem;

Json::Value readJson(fs::path jsonfile) {
  // occasionally it fails, presumably when the json isn't done being written. so repeat 10 times
  std::ifstream infile(jsonfile.string().c_str());
  if (infile.fail()) throw FileOpenError(jsonfile.string());

  Json::Reader reader;
  Json::Value root;
  infile >> root;
  return root;
}

float randf() {return (float)rand()/(float)RAND_MAX;}

void makeFullyTransparent(BulletObject::Ptr obj) {
  osg::Depth* depth = new osg::Depth;
  depth->setWriteMask(false);
  obj->node->getOrCreateStateSet()->setAttributeAndModes(depth, osg::StateAttribute::ON);
}

struct LocalConfig: Config {
  static int nSteps;
  static int nIter;
  static string probSpec;

  LocalConfig() :
    Config() {
    params.push_back(new Parameter<int> ("nSteps", &nSteps, "n samples of trajectory"));
    params.push_back(new Parameter<int> ("nIter", &nIter, "num iterations"));
    params.push_back(new Parameter<string> ("probSpec", &probSpec, "problem specification"));
  }
};
int LocalConfig::nSteps = 100;
int LocalConfig::nIter = 100;
string LocalConfig::probSpec = "";

void removeBodiesFromBullet(vector<BulletObject::Ptr> objs, btDynamicsWorld* world) {
  BOOST_FOREACH(BulletObject::Ptr obj, objs) {
    if (obj && obj->rigidBody)
      world->removeRigidBody(obj->rigidBody.get());
  }
}

int main(int argc, char *argv[]) {

  BulletConfig::linkPadding = .02;
  GeneralConfig::verbose=20000;
  GeneralConfig::scale = 10.;

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(LocalConfig());
  parser.addGroup(SQPConfig());
  parser.read(argc, argv);

  if (GeneralConfig::verbose > 0) getGRBEnv()->set(GRB_IntParam_OutputFlag, 0);

  Scene scene;
  scene.startViewer();

  util::setGlobalEnv(scene.env);


  Json::Value probInfo = readJson(LocalConfig::probSpec);

  if (probInfo.isMember("env")) Load(scene.env, scene.rave, probInfo["env"].asString());
  else ASSERT_FAIL();

  vector<double> startJoints;
  for (int i=0; i < probInfo["start_joints"].size(); ++i) startJoints.push_back(probInfo["start_joints"][i].asDouble());

  RaveRobotObject::Ptr pr2 = getRobotByName(scene.env, scene.rave, probInfo["robot"].asString());
  RaveRobotObject::Manipulator::Ptr arm = pr2->createManipulator(probInfo["manip"].asString(), false);

  assert(pr2);
  assert(arm);

  removeBodiesFromBullet(pr2->children, scene.env->bullet->dynamicsWorld);
  BOOST_FOREACH(EnvironmentObjectPtr obj, scene.env->objects) {
    BulletObjectPtr bobj = boost::dynamic_pointer_cast<BulletObject>(obj);
    obj->setColor(randf(),randf(),randf(),.4);
//    if (bobj) makeFullyTransparent(bobj);
  }
  pr2->setColor(0,1,1, .4);

  vector<double> goal;
  for (int i=0; i < probInfo["goal"].size(); ++i) goal.push_back(probInfo["goal"][i].asDouble());

  arm->setGripperAngle(.5);

  BulletRaveSyncher brs = syncherFromArm(arm);

  TIC();
  PlanningProblem prob;
  prob.addPlotter(ArmPlotterPtr(new ArmPlotter(arm, &scene,  SQPConfig::plotDecimation)));



  if (SQPConfig::pauseEachIter) {
    boost::function<void(PlanningProblem*)> func = boost::bind(&Scene::idle, &scene, true);
    prob.m_callbacks.push_back(func);
  }
  if (probInfo["goal_type"] == "joint") {
    int nJoints = 7;
    VectorXd startJoints = toVectorXd(arm->getDOFValues());
    VectorXd endJoints = toVectorXd(goal);
    MatrixXd initTraj = makeTraj(startJoints, endJoints, LocalConfig::nSteps);
    LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, true, defaultMaxStepMvmt(
        initTraj), SQPConfig::lengthCoef));
    CollisionCostPtr cc(new CollisionCost(pr2->robot, scene.env->bullet->dynamicsWorld, brs,
        arm->manip->GetArmIndices(), SQPConfig::distPen, SQPConfig::collCoefInit));
    JointBoundsPtr jb(new JointBounds(true, true, defaultMaxStepMvmt(initTraj) / 5, arm->manip));
    prob.initialize(initTraj, true);
    prob.addComponent(lcc);
    prob.addComponent(cc);
    prob.addComponent(jb);
  }
  else if (probInfo["goal_type"] == "cart") {

    VectorXd startJoints = toVectorXd(arm->getDOFValues());
    btTransform goalTrans = btTransform(btQuaternion(goal[0], goal[1], goal[2], goal[3]),
        btVector3(goal[4], goal[5], goal[6]));
    util::drawAxes(goalTrans, .15 * METERS, scene.env);
    TIC1();
    planArmToCartTarget(prob, startJoints, goalTrans, arm);
    cout << "total time: " << TOC();

  }
  else if (probInfo["goal_type"] == "grasp") {

    VectorXd startJoints = toVectorXd(arm->getDOFValues());
    btTransform goalTrans = btTransform(btQuaternion(goal[0], goal[1], goal[2], goal[3]),
        btVector3(goal[4], goal[5], goal[6]));
    util::drawAxes(goalTrans, .15 * METERS, scene.env);
    TIC1();
    planArmToGrasp(prob, startJoints, goalTrans, arm);
    cout << "total time: " << TOC();

  }


  prob.m_plotters[0].reset();

  BulletConfig::linkPadding = 0;
  scene.env->remove(pr2);
  PR2Manager pr2m1(scene);
  interactiveTrajPlot(prob.m_currentTraj, pr2m1.pr2->getManipByIndex(arm->index),  &scene);
  scene.idle(true);

}
