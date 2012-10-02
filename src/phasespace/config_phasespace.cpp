#include "config_phasespace.h"
#include "phasespace.h"
#include <boost/assign/list_of.hpp>

int PhasespaceConfig::validityNumFrames = 100;
float PhasespaceConfig::minConfidence = 1.0;
float PhasespaceConfig::frequency = OWL_MAX_FREQUENCY;
int PhasespaceConfig::maxPosHistory = 25;
float PhasespaceConfig::varianceTol = 0.000001;
string PhasespaceConfig::phasespaceTopic = "/phasespace_markers";

std::vector<std::string> PhasespaceConfig::cameraTopics = boost::assign::list_of("/kinect1")("/kinect2");
vector<string> PhasespaceConfig::kinectInfo_filenames = boost::assign::list_of("pr2head")("tripod");
vector<ledid_t> PhasespaceConfig::objLedIds = boost::assign::list_of(10)(9)(14)(11)(12)(8)(21)(22)(23) (13)(4)(5)(6)(7)(32)(33)(34)(35);
