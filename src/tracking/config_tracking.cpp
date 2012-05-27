#include "config_tracking.h"

std::string TrackingConfig::filteredCloudTopic = "/preprocessor/points";
std::string TrackingConfig::depthTopic  = "/camera/depth/image_rect";
std::string TrackingConfig::rgbTopic = "/camera/rgb/image_rect_color";
std::string TrackingConfig::fullCloudTopic = "/camera/rgb/points";

float TrackingConfig::outlierParam = 0;
float TrackingConfig::kp_rope = 1;
float TrackingConfig::kd_rope = 0;
float TrackingConfig::kp_cloth = 1;
float TrackingConfig::kd_cloth = 0;
