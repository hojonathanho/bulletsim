#include "sqp/sqp_ros.h"
#include "sqp/collision_map_tools.h"
#include "sqp/utils_ser.h"
#include "sqp/sqp_fwd.h"
#include "sqp/utils_sqp.h"
#include "utils/clock.h"
#include "utils/config.h"
#include "utils/conversions.h"
#include "utils/logging.h"
#include <boost/filesystem.hpp>
#include "simulation/util.h"
#include <bulletsim_msgs/PlanTraj.h>
#include "simulation/environment.h"
#include "simulation/openravesupport.h"
#include "clouds/cloud_ops.h"
#include "sqp/collision_boxes.h"
#include "sqp/collision_map_tools.h"
#include "sqp/traj_sqp.h"
#include "sqp/planning_problems2.h"
#include "sqp/config_sqp.h"
#include "sqp/plotters.h"


namespace fs = boost::filesystem;
using namespace util;
using namespace std;
using namespace bulletsim_msgs;

SQPROSConfig::SQPROSConfig() : Config() {
  params.push_back(new Parameter<string>("saveRoot", &saveRoot, "path to save problems to"));
  params.push_back(new Parameter<string>("loadProblem", &loadProblem, "load problem"));
  params.push_back(new Parameter<string>("robotFile", &robotFile, "robot file"));
  params.push_back(new Parameter<bool>("showFinalRun", &showFinalRun, "interactively show final trajectory"));
  params.push_back(new Parameter<float>("voxelSize", &voxelSize, "voxel size"));
  params.push_back(new Parameter<string>("collisionShape", &collisionShape, "shape: box, sphere, mesh"));
}
string SQPROSConfig::saveRoot = "/home/joschu/Data/planning";
string SQPROSConfig::loadProblem = "";
string SQPROSConfig::robotFile = "robots/pr2-beta-static.zae";
bool SQPROSConfig::showFinalRun;
float SQPROSConfig::voxelSize=.03;
string SQPROSConfig::collisionShape = "mesh";

bool validateRequest(const PlanTrajRequest& req) {
  bool valid = true;

	if (req.robot_joints.empty()) {
		LOG_ERROR("need to specify robot_joints");
		return false;
	}
  if (req.manip.empty()) {
    LOG_ERROR("no manip specified");
    valid = false;
  }
  if (req.task.empty()) {
    LOG_ERROR("no task specified");
    valid = false;
  }
  if (req.xmlstring.empty() && req.cloud.height*req.cloud.width==1) {
    LOG_WARN("planning with no point cloud or collision objects");
  }
  if (req.task.empty()) {
    LOG_ERROR("no task specified");
    valid = false;
  }
  if (req.goal.empty()) {
	LOG_ERROR("empty goal");
	valid = false;
  }
  
  return valid;
}

EnvironmentObjectPtr loadCloud(const sensor_msgs::PointCloud2& msg, Environment& env) {
  ColorCloudPtr fullCloud = fromROSMsg1(msg);
  ColorCloudPtr dsCloud = downsampleCloud(fullCloud, SQPROSConfig::voxelSize);
  EnvironmentObjectPtr collisionGeom;
  if (SQPROSConfig::collisionShape == "box") {
    collisionGeom = collisionBoxesFromPointCloud(dsCloud, SQPROSConfig::voxelSize);
  }
  else if (SQPROSConfig::collisionShape == "mesh") {
    collisionGeom = collisionMeshFromPointCloud(dsCloud, SQPROSConfig::voxelSize);
  }
  env.add(collisionGeom);
  return collisionGeom;
}



void removeNonRobotObjects(Environment& env, RaveInstance& rave) {
  typedef std::map<KinBodyPtr, RaveObject*> KB2RO;
  
  BOOST_FOREACH(const KB2RO::value_type& kb_ro, rave.rave2bulletsim) {
    if (!kb_ro.first->IsRobot()) {
      rave.env->Remove(kb_ro.first);
      env.remove(kb_ro.second->getEnvironmentObjectPtr());
    }
  }
}

bool writeStringToFile(const string& outstring, const string& fname) {
  ofstream outfile(fname.c_str());
  if (outfile.fail()) {
    LOG_ERROR("failed to write " << outfile);
    return false;
  }
  outfile << outstring;
  outfile.close();
  return outfile.fail();
}

bool setupEnvironment(const PlanTraj::Request& req, RaveRobotObject& robot, vector<EnvironmentObjectPtr>& addedObjs) {
    
    // load stuff from openrave xml file into robot's openrave-environment and bulletsim-environment.
    // append non-rave stuff to addedObjs list
    // added rave stuff isn't stored anywhere
    
  Environment& env = *robot.getEnvironment();
  RaveInstance& rave = *robot.rave.get();
  
  if (!req.xmlstring.empty()) {
    writeStringToFile(req.xmlstring, "/tmp/openrave_stuff.xml");
    Load(env.getEnvironmentPtr(), rave.getRaveInstancePtr(), "/tmp/openrave_stuff.xml");
  }
  
  if (req.cloud.width*req.cloud.height > 0) {
    addedObjs.push_back(loadCloud(req.cloud, env));
  }

  return true; 
}

void removeObjectsFromEnv(Environment& env, const vector<EnvironmentObjectPtr>& addedObjs) {
  BOOST_FOREACH(const EnvironmentObjectPtr& obj, addedObjs) {
    env.remove(obj);
  }
}


bool setupProblem(const PlanTraj::Request& req, TrajOptimizer& opt, RaveRobotObject& robot) {
  
	if (req.robot_joints.size() != robot.robot->GetDOF()) {
		LOG_ERROR_FMT("wrong number of joints! %i != %i", req.robot_joints.size(), robot.robot->GetDOF());
		return false;
	}
  robot.robot->SetDOFValues(req.robot_joints);
  if (!req.robot_transform.empty()) {
    vector<double> v = req.robot_transform;
    robot.robot->SetTransform(util::toRaveTransform(btTransform(btQuaternion(v[0], v[1], v[2], v[3]), btVector3(v[4], v[5], v[6]))));
  }
  robot.updateBullet();

  RobotManipulatorPtr manip = robot.createManipulator(req.manip);
  if (!manip) {
    LOG_ERROR("failed to load manipulator named "  << req.manip);
    return false;
  }

  if (SQPConfig::enablePlot) {
  	opt.m_plotters.push_back(ArmPlotterPtr(new ArmPlotter(manip, util::getGlobalScene(), SQPConfig::plotDecimation)));
  }

  if (req.task == "joint") {
    VectorXd goal = toVectorXd(req.goal);
    bool success = setupArmToJointTarget(opt, goal, manip);
    return success;
  }
  else if (req.task == "cart") {
    VectorXd goal = toVectorXd(req.goal);
    btTransform goalTrans = btTransform(btQuaternion(goal[0], goal[1], goal[2], goal[3]),
            btVector3(goal[4], goal[5], goal[6]));
    bool success = setupArmToCartTarget(opt, goalTrans, manip);
    return success;
  }
  else if (req.task == "grasp") {
    VectorXd goal = toVectorXd(req.goal);
    btTransform goalTrans = btTransform(btQuaternion(goal[0], goal[1], goal[2], goal[3]),
            btVector3(goal[4], goal[5], goal[6]));
    bool success = setupArmToCartTarget(opt, goalTrans, manip);
    return success;
  }
  else if (req.task == "follow_cart") {
    MatrixXd gs = Eigen::Map<const MatrixXd>(req.goal.data(), req.goal.size()/7, 7);
    vector<btTransform> transforms;
    for (int i=0; i < gs.rows(); ++i) transforms.push_back(
    		btTransform(btQuaternion(gs(i,0), gs(i,1), gs(i,2), gs(i,3)), btVector3(gs(i,4), gs(i,5), gs(i,6))));
    bool success = setupArmToFollowCart(opt, transforms, manip, KinBody::LinkPtr());
    return success;

  }
	LOG_ERROR("unrecognized task type " << req.task);
	return false;

}

void plotGoal(const PlanTrajRequest& req, vector<EnvironmentObjectPtr>& addedObjs) {
  if (req.task == "joint") {
  }
  else if (req.task == "cart" || req.task == "grasp") {
    VectorXd goal = toVectorXd(req.goal);
    btTransform goalTrans = btTransform(btQuaternion(goal[0], goal[1], goal[2], goal[3]),
            btVector3(goal[4], goal[5], goal[6]));
    addedObjs.push_back(util::drawAxes(goalTrans, .1*METERS, util::getGlobalEnv()));
  }
  else if (req.task == "follow_cart") {
    MatrixXd gs = Eigen::Map<const MatrixXd>(req.goal.data(), req.goal.size()/7, 7);
    vector<btTransform> transforms;
    for (int i=0; i < gs.rows(); ++i) {
  		btTransform goalTrans(btQuaternion(gs(i,0), gs(i,1), gs(i,2), gs(i,3)), btVector3(gs(i,4), gs(i,5), gs(i,6)));
      addedObjs.push_back(util::drawAxes(goalTrans, .1*METERS, util::getGlobalEnv()));
		}
  }
}

ColorCloudPtr transformPointCloud(ColorCloudPtr cloud, RobotBasePtr robot, const string& fromFrame, const string& toFrame) {
  if (!robot->GetLink(fromFrame)) {
    LOG_ERROR("not a link: " << fromFrame);
    return ColorCloudPtr();
  }
  if (!robot->GetLink(toFrame)) {
    LOG_ERROR("not a link: " << fromFrame);
    return ColorCloudPtr();
  }
  OpenRAVE::Transform raveTransform = robot->GetLink(toFrame)->GetTransform().inverse() * robot->GetLink(fromFrame)->GetTransform();
  Eigen::Affine3f tf = toEigenTransform(toBtTransform(raveTransform));
  return transformPointCloud1(cloud, tf);
}


void saveProblem(const string& probRoot, const PlanTraj::Request& req) {
  assert(fs::exists(probRoot));
  long unsigned int secs = (long unsigned int)GetClock()/1000;
  msgToFile(req, (fs::path(probRoot) / (boost::format("problem_%i")%secs).str()).string() );
}

void cleanup(RaveRobotObject& robot, const vector<EnvironmentObjectPtr>& addedObjs) {
  removeObjectsFromEnv(*robot.getEnvironment(), addedObjs);
  removeNonRobotObjects(*robot.getEnvironment(), *robot.rave);
}

bool handlePlanningRequest(const PlanTraj::Request& req, PlanTrajResponse& resp, RaveRobotObject& robot) {
  bool valid = validateRequest(req);
  if (!valid) return false;

  if (!SQPROSConfig::saveRoot.empty()) saveProblem(SQPROSConfig::saveRoot, req);
  vector<EnvironmentObjectPtr> addedObjs;
  setupEnvironment(req, robot, addedObjs);
  TrajOptimizer opt;
  bool setupSuccess = setupProblem(req, opt, robot);
  pauseScene();
  if (!setupSuccess) {
  	cleanup(robot, addedObjs);
  	return false;
  }
  
  plotGoal(req, addedObjs);

	opt.optimize(); // todo: should use outer optimization instead

	opt.m_plotters.clear();
	if (SQPROSConfig::showFinalRun) {
		interactiveTrajPlot(opt.m_traj, &robot, robot.robot->GetActiveDOFIndices(), util::getGlobalScene());
	}

  MatrixXd result = opt.m_traj;
  resp.trajectory = vector<double>();
  resp.trajectory.assign(result.data(), result.data() + result.rows()*result.cols());

  cleanup(robot, addedObjs);
  return true;

}
