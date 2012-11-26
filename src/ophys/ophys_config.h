#pragma once

#include "utils/config.h"
#include <Eigen/Dense>

struct OPhysConfig : public Config {
  static Eigen::Vector3d gravity;
  static double largeDt;
  static int interpPerTimestep;
  static double trustRadius;
};
