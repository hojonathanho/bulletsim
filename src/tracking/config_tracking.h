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
  
  static float pointPriorDist;
  static float pointOutlierDist;
  static float epsilon;
  static int normalizeIter;
  static float kp_rope;
  static float kd_rope;
  static float kp_cloth;
  static float kd_cloth;
  static float kp_box;
  static float kd_box;

  static int res_x;
  static int res_y;

    TrackingConfig() : Config() {
        params.push_back(new Parameter<std::string>("filteredCloudTopic", &filteredCloudTopic, "filtered cloud topic"));
        params.push_back(new Parameter<std::string>("depthTopic", &depthTopic, "depth image topic"));
        params.push_back(new Parameter<std::string>("rgbTopic", &rgbTopic, "rgb image topic"));
        params.push_back(new Parameter<std::string>("fullCloudTopic", &fullCloudTopic, "original point cloud topic topic"));

        params.push_back(new Parameter<float>("pointPriorDist", &pointPriorDist, "Prior distribution for xyz. This is also the initial values for sigmas xyz."));
        params.push_back(new Parameter<float>("pointOutlierDist", &pointOutlierDist, "Intuitively, observed points farther than distance=sqrt(3)*pointOutlierDist (special case) from a node are considered outliers. Pick distance/sqrt(3) ~= distance*0.5 ."));
        params.push_back(new Parameter<float>("epsilon", &epsilon, "Small normal to prevent terms going to infinity in the row normalization of estimateCorrespondence."));
        params.push_back(new Parameter<int>("normalizeIter", &normalizeIter, "Iterations of row/column normalization in the estimateCorrespondence algorithm."));
        params.push_back(new Parameter<float>("kp_rope", &kp_rope, "proportional gain for rope"));
        params.push_back(new Parameter<float>("kd_rope", &kd_rope, "damping for rope"));
        params.push_back(new Parameter<float>("kp_cloth", &kp_cloth, "proportional gain for cloth"));
        params.push_back(new Parameter<float>("kd_cloth", &kd_cloth, "damping for cloth"));
        params.push_back(new Parameter<float>("kp_box", &kp_box, "proportional gain for box"));
        params.push_back(new Parameter<float>("kd_box", &kd_box, "damping for box"));

        params.push_back(new Parameter<int>("res_x", &res_x, "towel resolution in x"));
        params.push_back(new Parameter<int>("res_y", &res_y, "towel resolution in y"));
    }
};
