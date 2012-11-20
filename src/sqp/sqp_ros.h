#pragma once
#include "utils/config.h"
#include <bulletsim_msgs/PlanTraj.h>
#include "simulation/simulation_fwd.h"
using std::string;

struct SQPROSConfig : Config {
  static string saveRoot;
  static string loadProblem;
  static string robotFile;
  static bool showFinalRun;
  static float voxelSize;

  SQPROSConfig();
};


bool handlePlanningRequest(const bulletsim_msgs::PlanTraj::Request& req,
		bulletsim_msgs::PlanTraj::Response& resp, RaveRobotObject& robot);
