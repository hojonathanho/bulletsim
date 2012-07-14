#pragma once
#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//Broadcasts the transform from the kinect_rgb_optical frame to the ground frame
//If kinect_rgb_optical has a grandparent (i.e. kinect_link), then a transform from the kinect_link frame to the ground frame is broadcasted such that the given transform is still as specified above
void broadcastKinectTransform(const btTransform& transform, const std::string& kinect_rgb_optical, const std::string& ground, tf::TransformBroadcaster& broadcaster, const tf::TransformListener& listener);
