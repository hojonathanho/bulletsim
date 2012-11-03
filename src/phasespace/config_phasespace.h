#pragma once
#include "utils/config.h"
#include <string>

struct PhasespaceConfig : Config {
  //Also defined in config_tracking.h
  typedef int16_t ledid_t;

  static int validityNumFrames;
  static float minConfidence;
  static float frequency;
  static int maxPosHistory;
  static float varianceTol;
  static std::string phasespaceTopic;
  static std::vector<std::string> cameraTopics;
  static std::vector<std::string> kinectInfo_filenames;
  static std::vector<ledid_t> objLedIds;

  PhasespaceConfig() : Config() {
    params.push_back(new Parameter<int>("validityNumFrames", &validityNumFrames, "The marker is considered invalid if its confidence has been less than minConfidence for more than validityNumFrames iterations"));
    params.push_back(new Parameter<float>("minConfidence", &minConfidence, "The position is not updated if the marker's confidence is less than this value."));
    params.push_back(new Parameter<float>("frequency", &frequency, "Streaming frequency from the phasespace server."));
    params.push_back(new Parameter<int>("maxPosHistory", &maxPosHistory, "Maximum number of positions saved for computing mean and variance of an LED."));
    params.push_back(new Parameter<float>("varianceTol", &varianceTol, "Variance tolerance. An LED is considered invalid for MarkerRigidStatic if its variance is greater than this number."));
    params.push_back(new Parameter<std::string>("phasespaceTopic", &phasespaceTopic, "Topic to publish/subscribe"));
    params.push_back(new ParameterVec<std::string>("cameraTopics", &cameraTopics, "Camera base topics. For calibration, only the first one is used."));
    params.push_back(new ParameterVec<std::string>("kinectInfo_filenames", &kinectInfo_filenames, "The rigid body infos are loaded/saved from this file. For calibration, only the first one is used."));
    params.push_back(new ParameterVec<ledid_t>("objLedIds", &objLedIds, "The order matters. Even. Match front back."));
  }
};
