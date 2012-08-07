#include "config_tracking.h"
#include "tracking_defs.h"

std::string TrackingConfig::filteredCloudTopic = "/preprocessor/points";
std::string TrackingConfig::depthTopic  = "/kinect1/depth_registered/image_rect";
std::string TrackingConfig::rgbTopic = "/kinect1/rgb/image_rect_color";
std::string TrackingConfig::fullCloudTopic = "/kinect1/rgb/points";

static const std::string cameraTopics_a[] = { "/kinect1" };
std::vector<std::string> TrackingConfig::cameraTopics = std::vector<std::string>(cameraTopics_a, cameraTopics_a+sizeof(cameraTopics_a)/sizeof(std::string));

static const int featureTypes_a[] = {FEAT_XYZ, FEAT_LAB};
std::vector<int> TrackingConfig::featureTypes = std::vector<int>(featureTypes_a, featureTypes_a+sizeof(featureTypes_a)/sizeof(int));


float TrackingConfig::pointPriorDist = 0.02;
float TrackingConfig::pointOutlierDist = 0.02;
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
