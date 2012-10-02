#pragma once
#include "utils/config.h"
#include <string>

struct SQPConfig : Config {
  static double collCoef;
  static double lengthCoef;
  static bool topCollOnly;
  SQPConfig() : Config() {
    params.push_back(new Parameter<double>("collCoef", &collCoef, "coeff for collision cost"));
    params.push_back(new Parameter<double>("lengthCoef", &lengthCoef, "coeff for quadratic length penalty"));
    params.push_back(new Parameter<bool>("topCollOnly", &topCollOnly, "if link has collision, don't bother with its children"));
  }
};
