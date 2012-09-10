#include "config_tracking.h"
#include "feature_extractor.h"

std::string TrackingConfig::filteredCloudTopic = "/preprocessor/points";
std::string TrackingConfig::depthTopic  = "/kinect1/depth_registered/image_rect";
std::string TrackingConfig::rgbTopic = "/kinect1/rgb/image_rect_color";
std::string TrackingConfig::fullCloudTopic = "/kinect1/depth_registered/points";

static const std::string cameraTopics_a[] = { "/kinect1" };
std::vector<std::string> TrackingConfig::cameraTopics = std::vector<std::string>(cameraTopics_a, cameraTopics_a+sizeof(cameraTopics_a)/sizeof(std::string));

static const int featureTypes_a[] = {FeatureExtractor::FT_XYZ, FeatureExtractor::FT_LAB};
std::vector<int> TrackingConfig::featureTypes = std::vector<int>(featureTypes_a, featureTypes_a+sizeof(featureTypes_a)/sizeof(int));


float TrackingConfig::pointOutlierDist = 0.02;
float TrackingConfig::pointPriorDist = 0.02;
float TrackingConfig::colorLPriorDist = 0.4;
float TrackingConfig::colorABPriorDist = 0.08;
float TrackingConfig::epsilon = 0.001;
int TrackingConfig::normalizeIter = 2;
float TrackingConfig::kp_rope = 1;
float TrackingConfig::kd_rope = 0;
float TrackingConfig::kp_cloth = 1500;
float TrackingConfig::kd_cloth = 15;
float TrackingConfig::kp_box = 15;
float TrackingConfig::kd_box = 0.2;

float TrackingConfig::tracked_node_distance = 0.05;
float TrackingConfig::node_distance = 0.01; // 1 cm between nodes
float TrackingConfig::surface_density = 0.01/(0.1*0.1); // 10 grams per 100 cm2
int TrackingConfig::node_pixel = 10;
