#pragma once
// histogram based plane finding
// because the usual ransac procedure fails when you have big walls


#include "clouds/utils_pcl.h"

float getTableHeight(ColorCloudPtr cloud);