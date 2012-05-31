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
#include "simulation/bullet_io.h"
#include "simulation/softbodies.h"
#include "utils/logging.h"

using namespace std;

TrackedObject::Ptr toTrackedObject(const bulletsim_msgs::ObjectInit& initMsg, Environment::Ptr env) {
  if (initMsg.type == "rope") {
	  vector<btVector3> nodes = toBulletVectors(initMsg.rope.nodes);
	  BOOST_FOREACH(btVector3& node, nodes) node += btVector3(0,0,.01);
	  CapsuleRope::Ptr rope(new CapsuleRope(scaleVecs(nodes,METERS), initMsg.rope.radius*METERS));
	  return TrackedObject::Ptr(new TrackedRope(rope));
  }
  else if (initMsg.type == "towel_corners") {
	  const vector<geometry_msgs::Point32>& points = initMsg.towel_corners.polygon.points;
	  vector<btVector3> corners = scaleVecs(toBulletVectors(points),METERS);
	  BulletSoftObject::Ptr sim = makeTowel(corners, env->bullet->softBodyWorldInfo);
	  assert(!!sim);
	  return TrackedTowel::Ptr(new TrackedTowel(sim, 45, 31));
  }
  else
	  throw runtime_error("unrecognized initialization type" + initMsg.type);
}

bulletsim_msgs::TrackedObject toTrackedObjectMessage(TrackedObject::Ptr obj) {
  bulletsim_msgs::TrackedObject msg;
  if (obj->m_type == "rope") {
    msg.type = obj->m_type;
    msg.rope.nodes = toROSPoints(obj->getPoints());
  }
  else {
	  LOG_ERROR("I don't knot how to publish a ");
  }
  return msg;
}

TrackedObject::Ptr callInitServiceAndCreateObject(ColorCloudPtr cloud, Environment::Ptr env) {
  bulletsim_msgs::Initialization init;
  pcl::toROSMsg(*cloud, init.request.cloud);
  init.request.cloud.header.frame_id = "/ground";
	
  bool success = ros::service::call(initializationService, init);
  if (success)
	return toTrackedObject(init.response.objectInit, env);
  else {
	ROS_ERROR("initialization failed");
	return TrackedObject::Ptr();
  }

}
