#include "ravens_config.h"

int   RavenConfig::cloth = 0;
float RavenConfig::record_freq = 5.0;
int   RavenConfig::bcN = 5;
int   RavenConfig::bcM = 8;
float RavenConfig::bcS = 0.01;
float RavenConfig::bcH = 0.0015;

bool RavenConfig::enableLfd = false;
bool RavenConfig::useDemoLib = true;
bool RavenConfig::plotTfm = false;

// perturbations
char  RavenConfig::perturbFlap = 'r';
float RavenConfig::xBias = 0.0;
float RavenConfig::yBias = 0.0;
float RavenConfig::zBias = 0.0;
float RavenConfig::xRot = 0.0;
float RavenConfig::yRot = 0.0;
float RavenConfig::zRot = 0.0;

bool RavenConfig::ropeManip = false;
bool RavenConfig::holdEnd = true;
