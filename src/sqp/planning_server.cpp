#include "robots/robot_manager.h"
#include "simulation/simplescene.h"
#include "sqp/sqp_ros.h"
#include <bulletsim_msgs/PlanTraj.h>
#include <ros/ros.h>
#include "sqp/sqp.h"
#include "sqp/collisions.h"
#include "sqp/kinematics_utils.h"
#include "sqp/utils_ser.h"
#include <boost/filesystem.hpp>
#include "sqp/config_sqp.h"
namespace fs=boost::filesystem;
using namespace std;
using namespace bulletsim_msgs;

RaveRobotObjectPtr robot;

bool callback(PlanTrajRequest& req, PlanTrajResponse& resp) {
	return handlePlanningRequest(req, resp, *robot);
}

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(SQPConfig());
	parser.addGroup(SQPROSConfig());
	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.read(argc, argv);

	Scene scene;
  initializeGRB();
  util::setGlobalEnv(scene.env);
  util::setGlobalScene(&scene);
  setupBulletForSQP(scene.env->bullet->dynamicsWorld);
  Load(scene.env, scene.rave, SQPROSConfig::robotFile);
	RobotManager robotManager(scene);
  robot = robotManager.bot;
  removeBodiesFromBullet(robot->children, scene.env->bullet->dynamicsWorld);
  robot->setColor(1,1,1,.4);
  scene.startViewer();  
  
  if (SQPROSConfig::loadProblem.size() > 0) {
    PlanTraj::Request req = msgFromFile<PlanTrajRequest>((fs::path(SQPROSConfig::saveRoot) / SQPROSConfig::loadProblem / "req.msg").string());
    PlanTraj::Response resp;
    handlePlanningRequest(req, resp, *robot);
    scene.idle(true);
  }
  else {
    ros::init(argc, argv, "planning_server");
    ros::NodeHandle nh;
    
    ros::ServiceServer service = nh.advertiseService("plan_traj", callback);

    while (ros::ok()) {
      scene.draw();
      sleep(.05);
      ros::spinOnce();
    }
  }

}
