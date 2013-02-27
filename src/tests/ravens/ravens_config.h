#pragma once
#include "utils/config.h"
#include <btBulletDynamicsCommon.h>

struct RavenConfig : Config {
  static int cloth;
  static float record_freq;

  RavenConfig() : Config() {
    params.push_back(new Parameter<int>("cloth", &cloth, "include cloth in screen (1/0)"));
    params.push_back(new Parameter<float>("record_freq", &record_freq, "frequency of recording"));
  }

};
