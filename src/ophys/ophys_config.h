#include "utils/config.h"
#include <Eigen/Dense>
#pragma once

using namespace Eigen;

struct OPhysConfig : public Config {
  static Vector3d gravity;
  static double dt;
  static double trustRadius;
};
