#include "config_tracking.h"

std::string TrackingConfig::filteredCloudTopic = "/preprocessor/points";
std::string TrackingConfig::depthTopic  = "/kinect1/depth_registered/image_rect";
std::string TrackingConfig::rgbTopic = "/kinect1/rgb/image_rect_color";
std::string TrackingConfig::fullCloudTopic = "/kinect1/rgb/points";

std::string cameraTopics_a[] = { "/kinect1" };
std::vector<std::string> TrackingConfig::cameraTopics = std::vector<std::string>(cameraTopics_a, cameraTopics_a+sizeof(cameraTopics_a)/sizeof(std::string));

float TrackingConfig::pointPriorDist = 0.02;
float TrackingConfig::pointOutlierDist = 0.02;
float TrackingConfig::epsilon = 0.001;
int TrackingConfig::normalizeIter = 2;
float TrackingConfig::kp_rope = 1;
float TrackingConfig::kd_rope = 0;
float TrackingConfig::kp_cloth = 2000;
float TrackingConfig::kd_cloth = 50;
float TrackingConfig::kp_box = 15;
float TrackingConfig::kd_box = 0.2;

float TrackingConfig::surface_density = 0.01/(0.1*0.1); // 10 grams per 100 cm2
float TrackingConfig::node_density = 1/0.01; // 1 node per 1 cm
