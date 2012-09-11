// example1.cc
// simple point tracking program

#include <stdio.h>

#include "utils/config.h"
#include "owl.h"
#include <ros/ros.h>
#include "phasespace.h"
#include "phasespace_utils.h"
#include "config_phasespace.h"

// change these to match your configuration

#define SERVER_NAME "169.229.222.231"
#define INIT_FLAGS 0

void owl_print_error(const char *s, int n);

int main(int argc, char *argv[])
{
	Parser parser;
	parser.addGroup(PhasespaceConfig());
	parser.read(argc, argv);

	ros::init(argc, argv,"phasespace_simple_publisher");
	ros::NodeHandle nh;

  OWLMarker markers[MarkerSystemBase::m_marker_count];
  int tracker;

  if(owlInit(SERVER_NAME, INIT_FLAGS) < 0) return 0;

  // create tracker 0
  tracker = 0;
  owlTrackeri(tracker, OWL_CREATE, OWL_POINT_TRACKER);

  // set markers
  for(int i = 0; i < MarkerSystemBase::m_marker_count; i++)
    owlMarkeri(MARKER(tracker, i), OWL_SET_LED, i);

  // activate tracker
  owlTracker(tracker, OWL_ENABLE);

  // flush requests and check for errors
  if(!owlGetStatus()) {
		owl_print_error("error in point tracker setup", owlGetError());
		return 0;
	}

  // set default frequency
  owlSetFloat(OWL_FREQUENCY, PhasespaceConfig::frequency);

  // start streaming
  owlSetInteger(OWL_STREAMING, OWL_ENABLE);

  // rotate the origin by 90 degrees about the x-axis. This is because in graphics, the convention is to have y-axis pointing up.
  float pose[7] = { 0, 0, 0, sqrt(2.0)/2.0, sqrt(2.0)/2.0, 0, 0 };
  owlLoadPose(pose);

  // main loop
	boost::shared_ptr<ros::Publisher> publisher(new ros::Publisher(nh.advertise<bulletsim_msgs::OWLPhasespace>( PhasespaceConfig::phasespaceTopic, 5 )));
  ros::Rate rate(PhasespaceConfig::frequency);
  while (ros::ok()) {
		int err;

		// get some markers
		int n = owlGetMarkers(markers, MarkerSystemBase::m_marker_count);

		// check for error
		if((err = owlGetError()) != OWL_NO_ERROR) {
			owl_print_error("error", err);
			break;
		}

		// no data yet
		if(n == 0) { continue; }

		if(n > 0) {
//			printf("%d marker(s):\n", n);
			for(int i = 0; i < n; i++) {
				bulletsim_msgs::OWLPhasespace phasespace_msg;
				for (int ind=0; ind<MarkerSystemBase::m_marker_count; ind++)
					phasespace_msg.markers.push_back(toOWLMarkerMsg(markers[ind]));
				publisher->publish(phasespace_msg);

//				if(markers[i].cond > 0)
//					printf("%d) %.4f %f %f %f\n", i, markers[i].cond, markers[i].x, markers[i].y, markers[i].z);
			}
//			printf("\n");
		}

		ros::spinOnce();
    rate.sleep();
  }

  // cleanup
  owlDone();

  return 0;
}

void owl_print_error(const char *s, int n)
{
  if(n < 0) printf("%s: %d\n", s, n);
  else if(n == OWL_NO_ERROR) printf("%s: No Error\n", s);
  else if(n == OWL_INVALID_VALUE) printf("%s: Invalid Value\n", s);
  else if(n == OWL_INVALID_ENUM) printf("%s: Invalid Enum\n", s);
  else if(n == OWL_INVALID_OPERATION) printf("%s: Invalid Operation\n", s);
  else printf("%s: 0x%x\n", s, n);
}
