#include <ros/topic.h>
#include <ros/console.h>
#include "initialization.h"
#include "utils_tracking.h"
#include "config_tracking.h"
#include <geometry_msgs/Transform.h>
#include "simulation/config_bullet.h"
#include "utils/conversions.h"
#include <bulletsim_msgs/TrackedObject.h>
#include <bulletsim_msgs/Initialization.h>
#include <pcl/ros/conversions.h>
#include "tracked_object.h"
#include <tf/tf.h>

using namespace std;

TrackedObject::Ptr toTrackedObject(const bulletsim_msgs::ObjectInit& initMsg) {
  if (initMsg.type == "rope") {
	  vector<btVector3> nodes = toBulletVectors(initMsg.rope.nodes);
	  CapsuleRope::Ptr rope(new CapsuleRope(scaleBy(nodes,METERS), initMsg.rope.radius*METERS));
	  return TrackedObject::Ptr(new TrackedRope(rope));
  }
  else if (initMsg.type == "towel") {
    throw runtime_error("not implemented yet");
  }
}

bulletsim_msgs::TrackedObject toTrackedObjectMessage(TrackedObject::Ptr obj) {
  if (obj->m_type == "rope") {
	bulletsim_msgs::TrackedObject msg;
    msg.type = obj->m_type;
    if (obj->m_type == "rope") {
        msg.rope.nodes = toROSPoints(toBulletVectors(obj->getPoints()));
    }
    else throw runtime_error("invalid object type");
  }
}

TrackedObject::Ptr callInitServiceAndCreateObject(ColorCloudPtr cloud) {
	bulletsim_msgs::Initialization init;
  pcl::toROSMsg(*cloud, init.request.cloud);
	
	bool success = ros::service::call(initializationService, init);
	if (success)
		return toTrackedObject(init.response.objectInit);
	else {
		ROS_ERROR("initialization failed");
		return TrackedObject::Ptr();
	}

}
