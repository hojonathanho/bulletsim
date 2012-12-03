#include "ophys_config.h"

using namespace Eigen;

double OPhysConfig::gravity = -9.8;
double OPhysConfig::largeDt = 0.5;
int OPhysConfig::interpPerTimestep = 10;
double OPhysConfig::trustRadius = 0.05;
int OPhysConfig::N = 2;
int OPhysConfig::T = 5;
bool OPhysConfig::runTests = true;
double OPhysConfig::tableHeight = 0.7;
double OPhysConfig::tableWidth = 2.;
double OPhysConfig::tableLength = 2.;
double OPhysConfig::tableDistFromRobot = 0.2;
bool OPhysConfig::useRobot = false;

OPhysConfig::OPhysConfig() : Config() {
  params.push_back(new Parameter<double>("gravity", &gravity, "gravity"));
  params.push_back(new Parameter<double>("largeDt", &largeDt, "coarse timestep"));
  params.push_back(new Parameter<int>("interpPerTimestep", &interpPerTimestep, "interpolation points per coarse timestep"));
  params.push_back(new Parameter<double>("trustRadius", &trustRadius, "trust radius (unused)"));
  params.push_back(new Parameter<int>("N", &N, "number of rope control points"));
  params.push_back(new Parameter<int>("T", &T, "number of coarse timesteps"));
  params.push_back(new Parameter<bool>("runTests", &runTests, "run tests"));

  params.push_back(new Parameter<double>("tableHeight", &tableHeight, ""));
  params.push_back(new Parameter<double>("tableWidth", &tableWidth, ""));
  params.push_back(new Parameter<double>("tableLength", &tableLength, ""));
  params.push_back(new Parameter<double>("tableDistFromRobot", &tableDistFromRobot, ""));

  params.push_back(new Parameter<bool>("useRobot", &useRobot, ""));
}
