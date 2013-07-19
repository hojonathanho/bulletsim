#pragma once
#include "utils/config.h"
#include <btBulletDynamicsCommon.h>

struct RavenConfig : Config {
  static int cloth;
  static float record_freq;

  static int bcN;
  static int bcM;
  static float bcS;
  static float bcH;

  static float xBias;
  static float yBias;
  static float zBias;

  static bool enableLfd;

  static bool plotTfm;

  static char biasAxis; // 'x', 'y' or 'z'
  static float biasAngle;

  static bool ropeManip;

  RavenConfig() : Config() {
    params.push_back(new Parameter<int>("cloth", &cloth, "include cloth in screen (1/0)"));
    params.push_back(new Parameter<float>("record_freq", &record_freq, "frequency of recording"));
	params.push_back(new Parameter<int>("bcN", &bcN, "BoxCloth: number of squares in x direction"));
    params.push_back(new Parameter<int>("bcM", &bcM, "BoxCloth: number of squares in y direction"));
    params.push_back(new Parameter<float>("bcS", &bcS, "BoxCloth: square side length"));
    params.push_back(new Parameter<float>("bcH", &bcH, "BoxCloth: box object height"));
    params.push_back(new Parameter<float>("xBias", &xBias, "Bias of box cloth in x dir"));
    params.push_back(new Parameter<float>("yBias", &yBias, "Bias of box cloth in y dir"));
    params.push_back(new Parameter<float>("zBias", &zBias, "Bias of box cloth in z dir"));
    params.push_back(new Parameter<bool>("enableLfd", &enableLfd, "enable learning from demonstrations for ravens"));
    params.push_back(new Parameter<bool>("plotTfm", &plotTfm, "bool for plotting transforms and paths"));

    params.push_back(new Parameter<char>("biasAxis",   &biasAxis, "axis to bias the suturing setup, in: {'x','y','z'}"));
    params.push_back(new Parameter<float>("biasAngle", &biasAngle, "bias angle [in degrees] for suturing setup"));

    params.push_back(new Parameter<bool>("ropeManip", &ropeManip, "bool to just have a rope in the scene."));

  }
};
