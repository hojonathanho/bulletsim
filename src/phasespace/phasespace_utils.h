#ifndef PHASESPACE_UTILS_H_
#define PHASESPACE_UTILS_H_

#include <vector>
#include <owl.h>
#include <bulletsim_msgs/OWLMarker.h>
#include <bulletsim_msgs/OWLPhasespace.h>

OWLMarker toOWLMarker(bulletsim_msgs::OWLMarker marker_msg);
bulletsim_msgs::OWLMarker toOWLMarkerMsg(OWLMarker marker);

#endif /* PHASESPACE_UTILS_H_ */
