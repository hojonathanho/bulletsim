#include "config_tracking.h"

std::string TrackingConfig::filteredCloudTopic = "/preprocessor/points";
std::string TrackingConfig::depthTopic  = "/kinect1/depth_registered/image_rect";
std::string TrackingConfig::rgbTopic = "/kinect1/rgb/image_rect_color";
std::string TrackingConfig::fullCloudTopic = "/kinect1/rgb/points";


std::string depthTopics_a[] = { "/kinect1/depth_registered/image_rect", "/kinect2/depth_registered/image_rect" };
std::vector<std::string> TrackingConfig::depthTopics = std::vector<std::string>(depthTopics_a, depthTopics_a+sizeof(depthTopics_a)/sizeof(std::string));

std::string rgbTopics_a[] = { "/kinect1/depth_registered/image_rect_color", "/kinect2/depth_registered/image_rect_color" };
std::vector<std::string> TrackingConfig::rgbTopics = std::vector<std::string>(rgbTopics_a, rgbTopics_a+sizeof(rgbTopics_a)/sizeof(std::string));

float TrackingConfig::pointPriorDist = 0.03;
float TrackingConfig::pointOutlierDist = 0.03;
float TrackingConfig::epsilon = 0.001;
int TrackingConfig::normalizeIter = 2;
float TrackingConfig::kp_rope = 1;
float TrackingConfig::kd_rope = 0;
float TrackingConfig::kp_cloth = 150;
float TrackingConfig::kd_cloth = 15;
float TrackingConfig::kp_box = 15;
float TrackingConfig::kd_box = 0.2;

int TrackingConfig::res_x = 45;
int TrackingConfig::res_y = 31;
