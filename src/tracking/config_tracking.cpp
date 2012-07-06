#include "config_tracking.h"

std::string TrackingConfig::filteredCloudTopic = "/preprocessor/points";
std::string TrackingConfig::depthTopic  = "/kinect1/depth_registered/image_rect";
std::string TrackingConfig::rgbTopic = "/kinect1/rgb/image_rect_color";
std::string TrackingConfig::fullCloudTopic = "/kinect1/rgb/points";

float TrackingConfig::pointPriorDist = 0.02;
float TrackingConfig::borderPriorDist = 0.5;
float TrackingConfig::pointOutlierDist = 0.02;
float TrackingConfig::borderOutlierDist = 0.5;
float TrackingConfig::epsilon = 0.001;
float TrackingConfig::kp_rope = 1;
float TrackingConfig::kd_rope = 0;
float TrackingConfig::kp_cloth = 150;
float TrackingConfig::kd_cloth = 15;
float TrackingConfig::kp_box = 15;
float TrackingConfig::kd_box = 0.2;

int TrackingConfig::res_x = 45;
int TrackingConfig::res_y = 31;
int TrackingConfig::fixeds = 1;
bool TrackingConfig::gendiags = false;
