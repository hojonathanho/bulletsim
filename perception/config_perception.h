#include "config.h"

struct TrackingConfig : Config {
  static float fAB;
  static float fBA;
  static int nIter;

  TrackingConfig() : Config() {
    params.push_back(new Parameter<float>("fAB", &fAB, "fAB"));
    params.push_back(new Parameter<float>("fBA", &fBA, "fBA"));
    params.push_back(new Parameter<int>("nIter", &nIter, "nIter"));
  }

};
