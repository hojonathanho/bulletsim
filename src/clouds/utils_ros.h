#pragma once
#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/JointState.h>

//Broadcasts the transform from the kinect_rgb_optical frame to the ground frame
//If kinect_rgb_optical has a grandparent (i.e. kinect_link), then a transform from the kinect_link frame to the ground frame is broadcasted such that the given transform is still as specified above
void broadcastKinectTransform(const btTransform& transform, const std::string& kinect_rgb_optical, const std::string& ground, tf::TransformBroadcaster& broadcaster, const tf::TransformListener& listener);

btTransform waitForAndGetTransform(const tf::TransformListener& listener, std::string target_frame, std::string source_frame);
bool lookupLatestTransform(btTransform& out, const std::string& toFrame, const std::string& fromFrame, tf::TransformListener& listener);

void synchronizeAndRegisterCallback(std::vector<std::string> cloud_topics, std::vector<std::string> image_topics, ros::NodeHandle nh, void (*callback)(const std::vector<sensor_msgs::PointCloud2ConstPtr>&, const std::vector<sensor_msgs::ImageConstPtr>&));

typedef message_filters::Subscriber<sensor_msgs::PointCloud2> CloudSub;
typedef boost::shared_ptr< CloudSub > CloudSubPtr;
typedef std::vector<CloudSubPtr> CloudSubPtrList;
typedef message_filters::Subscriber<sensor_msgs::JointState> JointSub;
typedef boost::shared_ptr<JointSub> JointSubPtr;

typedef void (*CloudAndJointCB)(const sensor_msgs::PointCloud2ConstPtr&, const sensor_msgs::JointStateConstPtr&);
void syncAndRegCloudJoint(std::string cloud_topic, ros::NodeHandle nh, CloudAndJointCB);
