#pragma once
#include <ros/ros.h>
#include <bulletsim_msgs/ObjectInit.h>
#include <bulletsim_msgs/TrackedObject.h>
#include "utils_tracking.h"
#include "tracked_object.h"
#include "clouds/utils_pcl.h"

TrackedObject::Ptr toTrackedObject(const bulletsim_msgs::ObjectInit& obj, CoordinateTransformer*);
bulletsim_msgs::TrackedObject toTrackedObjectMessage(TrackedObject::Ptr obj);
TrackedObject::Ptr callInitServiceAndCreateObject(ColorCloudPtr);
