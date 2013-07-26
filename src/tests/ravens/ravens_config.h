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

	static bool enableLfd;
	static bool useDemoLib;
	static bool plotTfm;

	// setup scene perturbations:
	static char  perturbFlap;
	static float xBias;
	static float yBias;
	static float zBias;
	static float xRot;
	static float yRot;
	static float zRot;


	static bool ropeManip;
	static bool holdEnd;

	RavenConfig() : Config() {
		params.push_back(new Parameter<int>("cloth", &cloth, "include cloth in screen (1/0)"));
		params.push_back(new Parameter<float>("record_freq", &record_freq, "frequency of recording"));
		params.push_back(new Parameter<int>("bcN", &bcN, "BoxCloth: number of squares in x direction"));
		params.push_back(new Parameter<int>("bcM", &bcM, "BoxCloth: number of squares in y direction"));
		params.push_back(new Parameter<float>("bcS", &bcS, "BoxCloth: square side length"));
		params.push_back(new Parameter<float>("bcH", &bcH, "BoxCloth: box object height"));

		params.push_back(new Parameter<bool>("enableLfd", &enableLfd, "enable learning from demonstrations for ravens"));
		params.push_back(new Parameter<bool>("plotTfm", &plotTfm, "bool for plotting transforms and paths"));

		// suture setup perturbations
		params.push_back(new Parameter<char>("perturbFlap", &perturbFlap, "which flap of the scene to perturb : ['r':right, 'l':left, 'b':both]"));
		params.push_back(new Parameter<float>("xBias", &xBias, "x-translation of suture setup"));
		params.push_back(new Parameter<float>("yBias", &yBias, "y-translation of suture setup"));
		params.push_back(new Parameter<float>("zBias", &zBias, "z-translation of suture setup"));
		params.push_back(new Parameter<float>("xRot", &xRot, "x-axis rotation angle [in degrees]"));
		params.push_back(new Parameter<float>("yRot", &yRot, "y-axis rotation angle [in degrees]"));
		params.push_back(new Parameter<float>("zRot", &zRot, "z-axis rotation angle [in degrees]"));


		params.push_back(new Parameter<bool>("ropeManip", &ropeManip, "bool to just have a rope in the scene"));
		params.push_back(new Parameter<bool>("holdEnd", &holdEnd, "bool to grab the other end of the rope"));

		params.push_back(new Parameter<bool>("useDemoLib", &useDemoLib, "Match up current scene against demos in the library. If false, uses playrunnum.txt"));


	}
};
