#pragma once
#include "utils/config.h"
#include <string>

struct SQPConfig : Config {
  static double collCoef;
  static double lengthCoef;
  SQPConfig() : Config() {
    params.push_back(new Parameter<double>("collCoef", &collCoef, "coeff for collision cost"));
    params.push_back(new Parameter<double>("lengthCoef", &lengthCoef, "coeff for quadratic length penalty"));
  }
};
