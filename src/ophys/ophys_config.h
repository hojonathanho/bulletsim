#pragma once

#include "utils/config.h"
#include <Eigen/Dense>

struct OPhysConfig : public Config {
  static double gravity;
  static double largeDt;
  static int interpPerTimestep;
  static double trustRadius;
  static int N;
  static int T;
  static bool runTests;

  OPhysConfig();
};
