#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/robot_manager.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "utils/vector_io.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "config_sqp.h"
#include <osg/Depth>
#include <json/json.h>
#include <boost/filesystem.hpp>
#include "utils/my_exceptions.h"
#include "kinematics_utils.h"
#include "plotters.h"
#include "traj_sqp.h"
#include "planning_problems2.h"
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


struct LocalConfig: Config {
  static string probSpec;
  static string jsonOutputPath;
  LocalConfig() :
    Config() {
    params.push_back(new Parameter<string> ("probSpec", &probSpec, "problem specification"));
    params.push_back(new Parameter<string> ("jsonOutputPath", &jsonOutputPath, "path to output final trajectory as JSON"));
  }
};
string LocalConfig::probSpec = "";
string LocalConfig::jsonOutputPath = "";

int main(int argc, char *argv[]) {

  BulletConfig::linkPadding = .02;
  BulletConfig::margin = 0;
  SQPConfig::padMult = 2;
  GeneralConfig::verbose=20000;
  GeneralConfig::scale = 10.;

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(LocalConfig());
  parser.addGroup(SQPConfig());
  parser.read(argc, argv);

  initializeGRB();


  Scene scene;
  scene.startViewer();

  util::setGlobalEnv(scene.env);
  util::setGlobalScene(&scene);
  scene.addVoidKeyCallback('=', boost::bind(&adjustWorldTransparency, .05), "increase opacity");
  scene.addVoidKeyCallback('-', boost::bind(&adjustWorldTransparency, -.05), "decrease opacity");


  Json::Value probInfo = readJson(LocalConfig::probSpec);

  if (probInfo.isMember("env")) Load(scene.env, scene.rave, probInfo["env"].asString());
  else ASSERT_FAIL();


  vector<double> startJoints;
  for (int i=0; i < probInfo["start_joints"].size(); ++i) startJoints.push_back(probInfo["start_joints"][i].asDouble());

  RaveRobotObject::Ptr robot = getRobotByName(scene.env, scene.rave, probInfo["robot"].asString());
  RaveRobotObject::Manipulator::Ptr arm = robot->createManipulator(probInfo["manip"].asString());

  assert(robot);
  assert(arm);

  setupBulletForSQP(scene.env->bullet->dynamicsWorld);
  removeBodiesFromBullet(robot->children, scene.env->bullet->dynamicsWorld);
  BOOST_FOREACH(EnvironmentObjectPtr obj, scene.env->objects) {
    BulletObjectPtr bobj = boost::dynamic_pointer_cast<BulletObject>(obj);
    obj->setColor(randf(),randf(),randf(),1);
//    if (bobj) makeFullyTransparent(bobj);
  }
  robot->setColor(0,1,1, .4);

  VectorXd goal(probInfo["goal"].size());
  for (int i=0; i < goal.size(); ++i) goal(i) = probInfo["goal"][i].asDouble();

  arm->setGripperAngle(.5);

  TIC();
  TrajOptimizer opt;
  opt.m_plotters.push_back(ArmPlotterPtr(new ArmPlotter(arm, &scene,  SQPConfig::plotDecimation)));


  if (probInfo["goal_type"] == "joint") {
    VectorXd startJoints = toVectorXd(arm->getDOFValues());
    bool success = setupArmToJointTarget(opt, goal, arm);
    assert(success);
  }
  else if (probInfo["goal_type"] == "cart") {
    btTransform goalTrans = btTransform(btQuaternion(goal[0], goal[1], goal[2], goal[3]),
            btVector3(goal[4], goal[5], goal[6]));
    bool success = setupArmToCartTarget(opt, goalTrans, arm);
    assert(success);
  }
  else if (probInfo["goal_type"] == "grasp") {
    btTransform goalTrans = btTransform(btQuaternion(goal[0], goal[1], goal[2], goal[3]),
            btVector3(goal[4], goal[5], goal[6]));
    util::drawAxes(goalTrans, .25*METERS, scene.env);
    bool success = setupArmToCartTarget(opt, goalTrans, arm);
    assert(success);
  }

//  checkAllLinearizations(opt);
//  opt.optimize();
  trajOuterOpt(opt, AllowedCollisions());

  if(!LocalConfig::jsonOutputPath.empty()){
//    prob.writeTrajToJSON(LocalConfig::jsonOutputPath);
  }

  opt.m_plotters[0].reset();

  BulletConfig::linkPadding = 0;
  interactiveTrajPlot(opt.m_traj, robot->getManipByIndex(arm->index),  &scene);
  scene.idle(true);

}
