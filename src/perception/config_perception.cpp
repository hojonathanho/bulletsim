#include "config_perception.h"

int TrackingConfig::nIter = 20;
float TrackingConfig::sigA = .025;
float TrackingConfig::sigB = .025;
float TrackingConfig::impulseSize = .5;
float TrackingConfig::cutoff = .01;
int TrackingConfig::nSamples = 1;
int TrackingConfig::stepsPerM = 2;
float TrackingConfig::kp = 1;
float TrackingConfig::kd = .2;

float TrackingConfig::outlierParam = .01;
bool TrackingConfig::showLines = false;
bool TrackingConfig::showObs = false;
bool TrackingConfig::showEst = false;
bool TrackingConfig::showKinect = true;
bool TrackingConfig::showSim = true;
bool TrackingConfig::startIdle = false;

float TrackingConfig::towelRes = 1;
float TrackingConfig::towelStiffness = 1;

string TrackingConfig::objType = "";
