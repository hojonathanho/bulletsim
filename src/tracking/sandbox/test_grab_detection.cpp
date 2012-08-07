#include "tracking/grab_detection.h"
#include <ros/ros.h>
#include <cstdio>

GrabDetector* lgd;
GrabDetector* rgd;

void grabCB() {
	printf("grab detected\n");
}


void jointCallback(const sensor_msgs::JointState& msg) {
	lgd->update(msg);
	rgd->update(msg);
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv,"test_grab_detection");
	ros::NodeHandle nh;
	ros::Subscriber jointSub = nh.subscribe("/joint_states", 1, jointCallback);
	lgd = new GrabDetector(GrabDetector::LEFT, &grabCB, &grabCB);
	rgd = new GrabDetector(GrabDetector::RIGHT, &grabCB, &grabCB);
	ros::spin();

}
