#include "phasespace_utils.h"

OWLMarker toOWLMarker(bulletsim_msgs::OWLMarker marker_msg) {
	OWLMarker marker;
	marker.id = marker_msg.id;
	marker.frame = marker_msg.frame;
	marker.x = marker_msg.point.x;
	marker.y = marker_msg.point.y;
	marker.z = marker_msg.point.z;
	marker.cond = marker_msg.cond;
	marker.flag = marker_msg.flag;
	return marker;
}

bulletsim_msgs::OWLMarker toOWLMarkerMsg(OWLMarker marker) {
	bulletsim_msgs::OWLMarker marker_msg;
	marker_msg.id = marker.id;
	marker_msg.frame = marker.frame;
	marker_msg.point.x = marker.x;
	marker_msg.point.y = marker.y;
	marker_msg.point.z = marker.z;
	marker_msg.cond = marker.cond;
	marker_msg.flag = marker.flag;
	return marker_msg;
}
