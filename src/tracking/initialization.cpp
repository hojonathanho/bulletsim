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
	  CapsuleRope::Ptr sim(new CapsuleRope(scaleVecs(nodes,METERS), initMsg.rope.radius*METERS));
	  env->add(sim);
//	  cv::Mat image = cv::imread("/home/alex/Desktop/image2.png");
//		if (TrackingConfig::fixeds==1) {
//			sim->setTexture(image);
//			cout << "texture 1" << endl;
//		}
//		if (TrackingConfig::gendiags) {
//			sim->setColor(1,0,0,1);
//			cout << "color 1" << endl;
//		}
//		if (TrackingConfig::fixeds==2) {
//			sim->setTexture(image);
//			cout << "texture 2" << endl;
//		}
	  return TrackedObject::Ptr(new TrackedRope(sim));
  }
  else if (initMsg.type == "towel_corners") {
	  const vector<geometry_msgs::Point32>& points = initMsg.towel_corners.polygon.points;
	  vector<btVector3> corners = scaleVecs(toBulletVectors(points),METERS);
	  int resolution_x = TrackingConfig::res_x;
	  int resolution_y = TrackingConfig::res_y;
//	  int resolution_x = 45;
//	  int resolution_y = 31;
	  BulletSoftObject::Ptr sim = makeTowel(corners, resolution_x, resolution_y, env->bullet->softBodyWorldInfo);
	  assert(!!sim);
	  env->add(sim);
	  cv::Mat image = cv::imread("/home/alex/Desktop/image.jpg");
	  if (TrackingConfig::fixeds==1) {
	  	sim->setTexture(image);
	  	cout << "texture 1" << endl;
	  }
	  if (TrackingConfig::gendiags) {
			sim->setColor(1,0,0,1);
	  	cout << "color 1" << endl;
	  }
	  if (TrackingConfig::fixeds==2) {
	  	sim->setTexture(image);
	  	cout << "texture 2" << endl;
	  }
	  return TrackedTowel::Ptr(new TrackedTowel(sim, resolution_x, resolution_y));
  }
  else if (initMsg.type == "box") {
	  btScalar mass = 1;
	  btVector3 halfExtents = toBulletVector(initMsg.box.extents)*0.5*METERS;
	  Eigen::Matrix3f rotation = (Eigen::Matrix3f) Eigen::AngleAxisf(initMsg.box.angle, Eigen::Vector3f::UnitZ());
	  btTransform initTrans(toBulletMatrix(rotation), toBulletVector(initMsg.box.center)*METERS);
	  BoxObject::Ptr sim(new BoxObject(mass, halfExtents, initTrans));
	  env->add(sim);
	  cv::Mat image = cv::imread("/home/alex/Desktop/image.jpg");
		sim->setTexture(image);
		//sim->setColor(1,0,0,1);
	  return TrackedBox::Ptr(new TrackedBox(sim));
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
