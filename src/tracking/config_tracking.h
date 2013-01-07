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
  
  static std::vector<std::string> cameraTopics;
  static std::vector<int> featureTypes;

  static float downsample;
  static bool applyEvidenceInit;

  static bool freeSpaceModel;
  static float pointOutlierDist;
  static float pointPriorCount;
  static float pointPriorDist;
  static float pointPriorDistNoObs;
  static float colorLPriorDist;
  static float colorABPriorDist;
  static float epsilon;
  static int normalizeIter;
  static float kp_rope;
  static float kd_rope;
  static float kp_cloth;
  static float kd_cloth;
  static float kp_box;
  static float kd_box;

  static float node_distance;
  static float node_density;
  static float surface_density;
  static int node_pixel;

  static float sponge_res;

  static std::string record_camera_pos_file;
  static std::string playback_camera_pos_file;

  static int dist;
  static float distParam;

    TrackingConfig() : Config() {
        params.push_back(new Parameter<std::string>("filteredCloudTopic", &filteredCloudTopic, "filtered cloud topic"));
        params.push_back(new Parameter<std::string>("depthTopic", &depthTopic, "depth image topic"));
        params.push_back(new Parameter<std::string>("rgbTopic", &rgbTopic, "rgb image topic"));
        params.push_back(new Parameter<std::string>("fullCloudTopic", &fullCloudTopic, "original point cloud topic topic"));

        params.push_back(new ParameterVec<std::string>("cameraTopics", &cameraTopics, "camera base topics"));
        params.push_back(new Parameter<float>("downsample", &downsample, "downsample after merging clouds"));
        params.push_back(new Parameter<bool>("applyEvidenceInit", &applyEvidenceInit, "If evidence should be applied upon start up"));
        params.push_back(new Parameter<bool>("freeSpaceModel", &freeSpaceModel, "Runs the new model in which nodes are also assigned to free space"));
        params.push_back(new Parameter<float>("pointPriorDist", &pointPriorDist, "Prior distribution for xyz. This is also the initial values for sigmas xyz. For cloth pick 0.08; for rope pick 0.02."));
        params.push_back(new Parameter<float>("pointPriorDistNoObs", &pointPriorDistNoObs, "No observation prior distribution for xyz. This is also the initial values for sigmas xyz."));
        params.push_back(new Parameter<float>("pointPriorCount", &pointPriorCount, "Number of pseudo observations"));
        params.push_back(new ParameterVec<int>("featureTypes", &featureTypes, "feature types. see feature_extractor.h"));
        params.push_back(new Parameter<float>("pointOutlierDist", &pointOutlierDist, "Intuitively, observed points farther than distance=sqrt(3)*pointOutlierDist (special case) from a node are considered outliers. Pick distance/sqrt(3) ~= distance*0.5 ."));
        params.push_back(new Parameter<float>("colorLPriorDist", &colorLPriorDist, "Prior distribution for the lightness term of the Lab feature."));
        params.push_back(new Parameter<float>("colorABPriorDist", &colorABPriorDist, "Prior distribution for the chromaticity terms (ab) of the Lab feature."));
        params.push_back(new Parameter<float>("epsilon", &epsilon, "Small normal to prevent terms going to infinity in the row normalization of estimateCorrespondence."));
        params.push_back(new Parameter<int>("normalizeIter", &normalizeIter, "Iterations of row/column normalization in the estimateCorrespondence algorithm."));
        params.push_back(new Parameter<float>("kp_rope", &kp_rope, "proportional gain for rope"));
        params.push_back(new Parameter<float>("kd_rope", &kd_rope, "damping for rope"));
        params.push_back(new Parameter<float>("kp_cloth", &kp_cloth, "proportional gain for cloth"));
        params.push_back(new Parameter<float>("kd_cloth", &kd_cloth, "damping for cloth"));
        params.push_back(new Parameter<float>("kp_box", &kp_box, "proportional gain for box"));
        params.push_back(new Parameter<float>("kd_box", &kd_box, "damping for box"));

        params.push_back(new Parameter<float>("node_density", &node_density, "# nodes per unit distance. (resolution-1)/length"));
        params.push_back(new Parameter<float>("surface_density", &surface_density, "surface density for towel. (total mass)/(total area)"));
        params.push_back(new Parameter<int>("node_pixel", &node_pixel, "pixels between nodes. nodes are the original nodes, not the decimated ones."));

        params.push_back(new Parameter<float>("sponge_res", &sponge_res, "sponge_res^3 is the maximum tetrahedra volume."));

        params.push_back(new Parameter<std::string>("record_camera_pos_file", &record_camera_pos_file, "file to save the camera positions to"));
        params.push_back(new Parameter<std::string>("playback_camera_pos_file", &playback_camera_pos_file, "file to read and playback camera positions from"));

        params.push_back(new Parameter<int>("dist", &dist, "Distribution type for stochastic tracker. 0:UNIFORM 1:NORMAL 2:NONE"));
        params.push_back(new Parameter<float>("distParam", &distParam, "Distribuition parameter for stochastic tracker. [-param,param] range for Uniform. variance = param for Normal."));
    }
};
