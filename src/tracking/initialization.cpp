#include <ros/topic.h>
#include <ros/console.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "initialization.h"
#include "utils_tracking.h"
#include "config_tracking.h"
#include <geometry_msgs/Transform.h>
#include "simulation/config_bullet.h"
#include "utils/conversions.h"
#include <bulletsim_msgs/TrackedObject.h>
#include <bulletsim_msgs/Initialization.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include "tracked_object.h"
#include <tf/tf.h>
#include "simulation/bullet_io.h"
#include "simulation/softbodies.h"
#include "utils/logging.h"

using namespace std;

TrackedObject::Ptr toTrackedObject(const bulletsim_msgs::ObjectInit& initMsg, ColorCloudPtr cloud, cv::Mat image, CoordinateTransformer* transformer) {
  if (initMsg.type == "rope") {
	  vector<btVector3> nodes = toBulletVectors(initMsg.rope.nodes);
//		//downsample nodes
//		vector<btVector3> nodes;
//		for (int i=0; i<nodes_o.size(); i+=3)
//			nodes.push_back(nodes_o[i]);
	  BOOST_FOREACH(btVector3& node, nodes) node += btVector3(0,0,.01);

	  CapsuleRope::Ptr sim(new CapsuleRope(scaleVecs(nodes,METERS), initMsg.rope.radius*METERS));
	  TrackedRope::Ptr tracked_rope(new TrackedRope(sim));
		cv::Mat tex_image = tracked_rope->makeTexture(cloud);
		sim->setTexture(tex_image);

	  return tracked_rope;
  }
  else if (initMsg.type == "towel_corners") {
	  const vector<geometry_msgs::Point32>& points = initMsg.towel_corners.polygon.points;
	  vector<btVector3> corners = scaleVecs(toBulletVectors(points),METERS);
	  int resolution_x, resolution_y;
	  BulletSoftObject::Ptr sim = makeTowel(corners, TrackingConfig::node_density/METERS, TrackingConfig::surface_density/(METERS*METERS), resolution_x, resolution_y);
	  TrackedTowel::Ptr tracked_towel(new TrackedTowel(sim, resolution_x, resolution_y));
	  cv::Mat tex_image = tracked_towel->makeTexture(corners, image, transformer);
		sim->setTexture(tex_image);

	  return tracked_towel;
  }
  else if (initMsg.type == "box") {
	  btScalar mass = 1;
	  btVector3 halfExtents = toBulletVector(initMsg.box.extents)*0.5*METERS;
	  Eigen::Matrix3f rotation = (Eigen::Matrix3f) Eigen::AngleAxisf(initMsg.box.angle, Eigen::Vector3f::UnitZ());
	  btTransform initTrans(toBulletMatrix(rotation), (toBulletVector(initMsg.box.center) + btVector3(0,0,.15))*METERS);
	  BoxObject::Ptr sim(new BoxObject(mass, halfExtents, initTrans));
	  TrackedBox::Ptr tracked_box(new TrackedBox(sim));
		sim->setColor(1,0,0,1);

	  return tracked_box;
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
	  //TODO
	  //LOG_ERROR("I don't knot how to publish a ");
  }
  return msg;
}

TrackedObject::Ptr callInitServiceAndCreateObject(ColorCloudPtr cloud, cv::Mat image, CoordinateTransformer* transformer) {
  bulletsim_msgs::Initialization init;
  pcl::toROSMsg(*cloud, init.request.cloud);
  init.request.cloud.header.frame_id = "/ground";
	
  bool success = ros::service::call(initializationService, init);
  if (success)
  	return toTrackedObject(init.response.objectInit, scaleCloud(cloud,METERS), image, transformer);
  else {
		ROS_ERROR("initialization failed");
		return TrackedObject::Ptr();
  }

}
