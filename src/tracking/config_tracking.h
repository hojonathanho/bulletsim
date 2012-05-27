#pragma once
#include "utils/config.h"
#include <string>

static const std::string trackedObjectTopic = "/tracker/object";
static const std::string initializationService = "/initialization";

struct TrackingConfig : Config {
  static std::string filteredCloudTopic;
  static std::string depthTopic;
  static std::string rgbTopic;
  static std::string fullCloudTopic;
  
  static float outlierParam;
  static float kp_rope;
  static float kd_rope;
  static float kp_cloth;
  static float kd_cloth;

    TrackingConfig() : Config() {
        params.push_back(new Parameter<std::string>("filteredCloudTopic", &filteredCloudTopic, "filtered cloud topic"));
        params.push_back(new Parameter<std::string>("depthTopic", &depthTopic, "depth image topic"));
        params.push_back(new Parameter<std::string>("rgbTopic", &rgbTopic, "rgb image topic"));
        params.push_back(new Parameter<std::string>("fullCloudTopic", &fullCloudTopic, "original point cloud topic topic"));

        params.push_back(new Parameter<float>("outlierParam", &outlierParam, "outlier density"));
        params.push_back(new Parameter<float>("kp_rope", &kp_rope, "proportional gain for rope"));
        params.push_back(new Parameter<float>("kd_rope", &kd_rope, "damping for rope"));
        params.push_back(new Parameter<float>("kp_cloth", &kp_cloth, "proportional gain for cloth"));
        params.push_back(new Parameter<float>("kd_cloth", &kd_cloth, "damping flor cloth"));
    }
};
