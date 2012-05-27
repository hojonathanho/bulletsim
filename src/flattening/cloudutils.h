#ifndef _FL_CLOUDUTILS_H_
#define _FL_CLOUDUTILS_H_

#include "clouds/utils_pcl.h"

// sum of squared euclidean distances to nearest neighbors
// adapted from pcl::Registration::getFitnessScore()
double calcAlignmentScore(ColorCloudPtr input, ColorCloudPtr target, double max_range=std::numeric_limits<double>::max());


#endif // _FL_CLOUDUTILS_H_
