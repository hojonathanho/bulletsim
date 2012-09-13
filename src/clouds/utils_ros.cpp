#include "utils_ros.h"
#include "utils/logging.h"

using namespace std;
using sensor_msgs::Image;
using sensor_msgs::ImageConstPtr;
using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;

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

btTransform waitForAndGetTransform(const tf::TransformListener& listener, std::string target_frame, std::string source_frame) {
	tf::StampedTransform st;
	while(1) {
		try {
			listener.waitForTransform(target_frame, source_frame, ros::Time(0),ros::Duration(.1));
			listener.lookupTransform(target_frame, source_frame, ros::Time(0), st);
		} catch (...) {
			ROS_WARN("An exception was catched from waitForAndGetTransform. Retrying...");
			continue;
		}
		break;
	}
	return st.asBt();
}

bool lookupLatestTransform(btTransform& out, const std::string& toFrame, const std::string& fromFrame, tf::TransformListener& listener) {
  tf::StampedTransform st;
  try {
      listener.lookupTransform(toFrame, fromFrame, ros::Time(0), st);
  }
  catch (tf::TransformException e) {
      LOG_WARN_FMT("tf error: %s\n", e.what());
      return false;
  }
  out = st.asBt();
  return true;
}

void vectorizeArgumentsAndInvoke(const PointCloud2ConstPtr& cloud_msgs0, const ImageConstPtr& image_msgs0, const ImageConstPtr& image_msgs1, void (*callback)(const vector<PointCloud2ConstPtr>&, const vector<ImageConstPtr>&)) {
	vector<PointCloud2ConstPtr> cloud_msgs;
	cloud_msgs.push_back(cloud_msgs0);
	vector<ImageConstPtr> image_msgs;
	image_msgs.push_back(image_msgs0);
	image_msgs.push_back(image_msgs1);
	(*callback)(cloud_msgs, image_msgs);
}

void vectorizeArgumentsAndInvoke(const PointCloud2ConstPtr& cloud_msgs0, const PointCloud2ConstPtr& cloud_msgs1, const ImageConstPtr& image_msgs0, const ImageConstPtr& image_msgs1, const ImageConstPtr& image_msgs2, const ImageConstPtr& image_msgs3, void (*callback)(const vector<PointCloud2ConstPtr>&, const vector<ImageConstPtr>&)) {
	vector<PointCloud2ConstPtr> cloud_msgs;
	cloud_msgs.push_back(cloud_msgs0);
	cloud_msgs.push_back(cloud_msgs1);
	vector<ImageConstPtr> image_msgs;
	image_msgs.push_back(image_msgs0);
	image_msgs.push_back(image_msgs1);
	image_msgs.push_back(image_msgs2);
	image_msgs.push_back(image_msgs3);
	(*callback)(cloud_msgs, image_msgs);
}

void synchronizeAndRegisterCallback(std::vector<std::string> cloud_topics, std::vector<std::string> image_topics, ros::NodeHandle nh, void (*callback)(const std::vector<PointCloud2ConstPtr>&, const std::vector<ImageConstPtr>&))
{
	vector<message_filters::Subscriber<PointCloud2>*> cloud_subs(cloud_topics.size(), NULL);
	for (int i=0; i<cloud_topics.size(); i++)
		cloud_subs[i] = new message_filters::Subscriber<PointCloud2>(nh, cloud_topics[i], 1);

	vector<message_filters::Subscriber<Image>*> image_subs(image_topics.size(), NULL);
	for (int i=0; i<image_topics.size(); i++)
		image_subs[i] = new message_filters::Subscriber<Image>(nh, image_topics[i], 1);

  if (cloud_topics.size()==1 && image_topics.size()==2) {
  	typedef message_filters::sync_policies::ApproximateTime<PointCloud2, Image, Image> ApproxSyncPolicy;
  	message_filters::Synchronizer<ApproxSyncPolicy>* sync = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(30), *(cloud_subs[0]), *(image_subs[0]), *(image_subs[1]));
		sync->registerCallback(boost::bind(vectorizeArgumentsAndInvoke,_1,_2,_3,callback));
  } else if (cloud_topics.size()==2 && image_topics.size()==4) {
  	typedef message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2, Image, Image, Image, Image> ApproxSyncPolicy;
		message_filters::Synchronizer<ApproxSyncPolicy>* sync = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(30), *(cloud_subs[0]), *(cloud_subs[1]), *(image_subs[0]), *(image_subs[1]), *(image_subs[2]), *(image_subs[3]));
		sync->registerCallback(boost::bind(vectorizeArgumentsAndInvoke,_1,_2,_3,_4,_5,_6,callback));
  } else {
  	runtime_error("This case for synchronizeAndRegisterCallback is not implemented yet.");
  }
}
