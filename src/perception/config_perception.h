#pragma once
#include "utils/config.h"

struct TrackingConfig : Config {
  static int nIter;
  static float sigA;
  static float sigB;
  static float impulseSize;
  static float cutoff;
  static int nSamples;
  static float outlierParam;
  static bool showLines;
  static bool showObs;
  static bool showEst;
  static bool showKinect;
  static bool showSim;
  static bool startIdle;
  static float towelRes;
  static float towelStiffness;
  
  TrackingConfig() : Config() {
    params.push_back(new Parameter<int>("nIter", &nIter, "nIter"));
    params.push_back(new Parameter<float>("sigA", &sigA, "variance on estimated positions"));
    params.push_back(new Parameter<float>("sigB", &sigB, "variance on observed positions"));
    params.push_back(new Parameter<float>("impulseSize", &impulseSize, "scaling from log-lik gradient to impulse"));
    params.push_back(new Parameter<float>("cutoff", &cutoff, "smallest correspondence value"));
    params.push_back(new Parameter<float>("outlierParam", &cutoff, "p(outlier) * density"));
    params.push_back(new Parameter<int>("nSamples", &nSamples, "smallest correspondence value"));
    params.push_back(new Parameter<bool>("showLines",&showLines,"show force lines"));
    params.push_back(new Parameter<bool>("showObs",&showObs,"show downsampled cloud"));
    params.push_back(new Parameter<bool>("showEst",&showEst,"show nodes"));
    params.push_back(new Parameter<bool>("showKinect",&showKinect,"show kinect point cloud"));
    params.push_back(new Parameter<bool>("showSim",&showSim,"show simulated objects"));
    params.push_back(new Parameter<bool>("startIdle",&startIdle, "start idle"));
    params.push_back(new Parameter<float>("towelRes",&towelRes, "towel resolution factor"));
    params.push_back(new Parameter<float>("towelStiffness",&towelStiffness, "towel stiffness"));
  }
};

