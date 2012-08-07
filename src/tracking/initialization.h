#pragma once
#include <ros/ros.h>
#include <bulletsim_msgs/ObjectInit.h>
#include <bulletsim_msgs/TrackedObject.h>
#include "utils_tracking.h"
#include "tracked_object.h"
#include "clouds/utils_pcl.h"
#include "utils_tracking.h"

TrackedObject::Ptr toTrackedObject(const bulletsim_msgs::ObjectInit& obj);
bulletsim_msgs::TrackedObject toTrackedObjectMessage(TrackedObject::Ptr obj);
TrackedObject::Ptr callInitServiceAndCreateObject(ColorCloudPtr, cv::Mat, CoordinateTransformer*);
