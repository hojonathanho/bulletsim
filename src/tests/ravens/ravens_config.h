#pragma once
#include "utils/config.h"
#include <btBulletDynamicsCommon.h>

struct RavenConfig : Config {
  static int cloth;
  static float record_freq;
  static int lfdProcessing;

  static int bcN;
  static int bcM;
  static float bcS;
  static float bcH;

  RavenConfig() : Config() {
    params.push_back(new Parameter<int>("cloth", &cloth, "include cloth in screen (1/0)"));
    params.push_back(new Parameter<float>("record_freq", &record_freq, "frequency of recording"));
    params.push_back(new Parameter<int>("lfdProcessing", &lfdProcessing, "perform lfd processing"));
	params.push_back(new Parameter<int>("bcN", &bcN, "BoxCloth: number of squares in x direction"));
    params.push_back(new Parameter<int>("bcM", &bcM, "BoxCloth: number of squares in y direction"));
    params.push_back(new Parameter<float>("bcS", &bcS, "BoxCloth: square side length"));
    params.push_back(new Parameter<float>("bcH", &bcH, "BoxCloth: box object height"));

  }

};
