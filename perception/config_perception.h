#pragma once
#include "config.h"

struct TrackingConfig : Config {
  static int nIter;
  static float sigA;
  static float sigB;
  static float stepSize;
  static float cutoff;
  static int nSamples;
  static float outlierParam;

  TrackingConfig() : Config() {
    params.push_back(new Parameter<int>("nIter", &nIter, "nIter"));
    params.push_back(new Parameter<float>("sigA", &sigA, "variance on estimated positions"));
    params.push_back(new Parameter<float>("sigB", &sigB, "variance on observed positions"));
    params.push_back(new Parameter<float>("stepSize", &stepSize, "scaling from log-lik gradient to impulse"));
    params.push_back(new Parameter<float>("cutoff", &cutoff, "smallest correspondence value"));
    params.push_back(new Parameter<float>("outlierParam", &cutoff, "p(outlier) * density"));
    params.push_back(new Parameter<int>("nSamples", &nSamples, "smallest correspondence value"));
  }

};

