#pragma once
#include <ros/ros.h>
#include <bulletsim_msgs/ObjectInit.h>
#include <bulletsim_msgs/TrackedObject.h>
#include "utils_tracking.h"
#include "tracked_object.h"
#include "clouds/utils_pcl.h"
#include "utils_tracking.h"

EnvironmentObject::Ptr toInitializedObject(const bulletsim_msgs::ObjectInit& obj);
bulletsim_msgs::TrackedObject toTrackedObjectMessage(TrackedObject::Ptr obj);
EnvironmentObject::Ptr callInitServiceAndCreateObject(ColorCloudPtr cloud, cv::Mat image, cv::Mat mask, CoordinateTransformer* transformer);
