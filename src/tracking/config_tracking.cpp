#include "config_tracking.h"
#include "feature_extractor.h"
#include <boost/assign.hpp>

std::string TrackingConfig::filteredCloudTopic = "/preprocessor/points";
std::string TrackingConfig::depthTopic  = "/kinect1/depth_registered/image_rect";
std::string TrackingConfig::rgbTopic = "/kinect1/rgb/image_rect_color";
std::string TrackingConfig::fullCloudTopic = "/kinect1/depth_registered/points";

std::vector<std::string> TrackingConfig::cameraTopics = boost::assign::list_of("/kinect1");
std::vector<int> TrackingConfig::featureTypes = boost::assign::list_of(FeatureExtractor::FT_XYZ)(FeatureExtractor::FT_LAB);

float TrackingConfig::downsample = 0.02;

float TrackingConfig::pointOutlierDist = 0.02;
float TrackingConfig::pointPriorCount = 10;
float TrackingConfig::pointPriorDist = 0.02;
float TrackingConfig::colorLPriorDist = 0.4;
float TrackingConfig::colorABPriorDist = 0.08;
float TrackingConfig::epsilon = 0.001;
int TrackingConfig::normalizeIter = 2;
float TrackingConfig::kp_rope = 1;
float TrackingConfig::kd_rope = 0;
float TrackingConfig::kp_cloth = 1500;
float TrackingConfig::kd_cloth = 15;
float TrackingConfig::kp_box = 100;
float TrackingConfig::kd_box = 10;

float TrackingConfig::tracked_node_distance = 0.05;
float TrackingConfig::node_distance = 0.01; // 1 cm between nodes
float TrackingConfig::surface_density = 0.01/(0.1*0.1); // 10 grams per 100 cm2
int TrackingConfig::node_pixel = 10;

float TrackingConfig::sponge_res = 0.03;

std::string TrackingConfig::record_camera_pos_file = "";
std::string TrackingConfig::playback_camera_pos_file = "";
