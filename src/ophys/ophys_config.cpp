#include "ophys_config.h"

using namespace Eigen;

Vector3d OPhysConfig::gravity(0, 0, -9.8);
double OPhysConfig::largeDt = 0.5;
int OPhysConfig::interpPerTimestep = 10;
double OPhysConfig::trustRadius = 0.05;
