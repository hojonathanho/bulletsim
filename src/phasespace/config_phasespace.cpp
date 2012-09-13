#include "config_phasespace.h"
#include "phasespace.h"

int PhasespaceConfig::validityNumFrames = 100;
float PhasespaceConfig::minConfidence = 1.0;
float PhasespaceConfig::frequency = OWL_MAX_FREQUENCY;
int PhasespaceConfig::maxPosHistory = 25;
float PhasespaceConfig::varianceTol = 0.000001;
string PhasespaceConfig::phasespaceTopic = "/phasespace_markers";

static const std::string cameraTopics_a[] = { "/kinect1", "/kinect2" };
std::vector<std::string> PhasespaceConfig::cameraTopics = std::vector<std::string>(cameraTopics_a, cameraTopics_a+sizeof(cameraTopics_a)/sizeof(std::string));

static const string kinectInfo_filenames_a[] = { "pr2head", "tripod" };
vector<string> PhasespaceConfig::kinectInfo_filenames = std::vector<string>(kinectInfo_filenames_a, kinectInfo_filenames_a+sizeof(kinectInfo_filenames_a)/sizeof(string));

static const ledid_t objLedIds_a[] = { 10,9,14,11,12,8,21,22,23, 13,4,5,6,7,32,33,34,35 };
vector<ledid_t> PhasespaceConfig::objLedIds = std::vector<ledid_t>(objLedIds_a, objLedIds_a+sizeof(objLedIds_a)/sizeof(ledid_t));
