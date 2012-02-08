#include "config_perception.h"
int TrackingConfig::nIter = 20;
float TrackingConfig::sigA = .025;
float TrackingConfig::sigB = .025;
float TrackingConfig::impulseSize = .5;
float TrackingConfig::cutoff = .01;
int TrackingConfig::nSamples = 7;
float TrackingConfig::outlierParam = .01;
