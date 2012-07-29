#include "utils_ros.h"

//Broadcasts the transform from the kinect_rgb_optical frame to the ground frame
//If kinect_rgb_optical has a grandparent (i.e. kinect_link), then a transform from the kinect_link frame to the ground frame is broadcasted such that the given transform is still as specified above
void broadcastKinectTransform(const btTransform& transform, const std::string& kinect_rgb_optical, const std::string& ground, tf::TransformBroadcaster& broadcaster, const tf::TransformListener& listener) {
	std::string link;
	if (listener.getParent(kinect_rgb_optical, ros::Time(0), link) && listener.getParent(link, ros::Time(0), link)) {
			tf::StampedTransform tfTransform;
			listener.lookupTransform (link, kinect_rgb_optical, ros::Time(0), tfTransform);
			broadcaster.sendTransform(tf::StampedTransform(transform * tfTransform.asBt().inverse(), ros::Time::now(), ground, link));
	} else {
			broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), ground, kinect_rgb_optical));
	}
}
