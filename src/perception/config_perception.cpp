#include "config_perception.h"

int TrackingConfig::nIter = 20;
float TrackingConfig::sigA = .025;
float TrackingConfig::sigB = .025;
float TrackingConfig::impulseSize = .5;
float TrackingConfig::cutoff = .01;
int TrackingConfig::nSamples = 7;
float TrackingConfig::outlierParam = .01;
bool TrackingConfig::showLines = false;
bool TrackingConfig::showObs = false;
bool TrackingConfig::showEst = false;
bool TrackingConfig::showKinect = true;
bool TrackingConfig::showSim = true;
bool TrackingConfig::startIdle = false;
float TrackingConfig::towelRes = 1;
float TrackingConfig::towelStiffness = 1;
