#include "config_tracking.h"

std::string TrackingConfig::filteredCloudTopic = "/preprocessor/points";
std::string TrackingConfig::depthTopic  = "/kinect1/depth_registered/image_rect";
std::string TrackingConfig::rgbTopic = "/kinect1/rgb/image_rect_color";
std::string TrackingConfig::fullCloudTopic = "/kinect1/rgb/points";

float TrackingConfig::outlierParam = .1;
float TrackingConfig::kp_rope = 1;
float TrackingConfig::kd_rope = 0;
float TrackingConfig::kp_cloth = 1;
float TrackingConfig::kd_cloth = 0;
float TrackingConfig::kp_box = 15;
float TrackingConfig::kd_box = 0.2;

int TrackingConfig::res_x = 3;
int TrackingConfig::res_y = 3;
int TrackingConfig::fixeds = 1+2;
bool TrackingConfig::gendiags = true;
