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
#include <pcl/kdtree/kdtree_flann.h>
#include "tracked_object.h"
#include <tf/tf.h>
#include "simulation/bullet_io.h"
#include "simulation/softbodies.h"
#include "utils/logging.h"

using namespace std;

TrackedObject::Ptr toTrackedObject(const bulletsim_msgs::ObjectInit& initMsg, ColorCloudPtr cloud, Environment::Ptr env) {
  if (initMsg.type == "rope") {
	  vector<btVector3> nodes = toBulletVectors(initMsg.rope.nodes);
	  BOOST_FOREACH(btVector3& node, nodes) node += btVector3(0,0,.01);
	  CapsuleRope::Ptr sim(new CapsuleRope(scaleVecs(nodes,METERS), initMsg.rope.radius*METERS));
	  env->add(sim);
	  TrackedObject::Ptr tracked_rope(new TrackedRope(sim));

//	  nodes = tracked_rope->getPoints();
//		cv::Mat image(1, nodes.size(), CV_8UC3);
//		for (int j=0; j<nodes.size(); j++) {
//			pcl::KdTreeFLANN<ColorPoint> kdtree;
//			kdtree.setInputCloud(cloud);
//			ColorPoint searchPoint;
//			searchPoint.x = nodes[j].x();
//			searchPoint.y = nodes[j].y();
//			searchPoint.z = nodes[j].z();
//			// Neighbors within radius search
//			float radius = ((float) TrackingConfig::fixeds)/10.0; //(fixeds in cm)
//			std::vector<int> pointIdxRadiusSearch;
//			std::vector<float> pointRadiusSquaredDistance;
//			float r,g,b;
//			r=g=b=0;
//			vector<unsigned char> R, G, B;
//			if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
//				for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++) {
//					r += cloud->points[pointIdxRadiusSearch[i]].r;
//					g += cloud->points[pointIdxRadiusSearch[i]].g;
//					b += cloud->points[pointIdxRadiusSearch[i]].b;
//					R.push_back(cloud->points[pointIdxRadiusSearch[i]].r);
//					G.push_back(cloud->points[pointIdxRadiusSearch[i]].g);
//					B.push_back(cloud->points[pointIdxRadiusSearch[i]].b);
//				}
//				r /= ((float) pointIdxRadiusSearch.size());
//				g /= ((float) pointIdxRadiusSearch.size());
//				b /= ((float) pointIdxRadiusSearch.size());
//			}
//			image.at<cv::Vec3b>(0,j)[0] = b;
//			image.at<cv::Vec3b>(0,j)[1] = g;
//			image.at<cv::Vec3b>(0,j)[2] = r;
//	//	  image.at<cv::Vec3b>(0,j)[0] = median(B);
//	//		image.at<cv::Vec3b>(0,j)[1] = median(G);
//	//		image.at<cv::Vec3b>(0,j)[2] = median(R);
//		}
//		sim->setTexture(image);

	  return tracked_rope;
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
	  TrackedObject::Ptr tracked_towel(new TrackedTowel(sim, resolution_x, resolution_y));

	  cv::Mat image = cv::imread("/home/alex/Desktop/image.jpg");
	  sim->setTexture(image);

	  return tracked_towel;
  }
  else if (initMsg.type == "box") {
	  btScalar mass = 1;
	  btVector3 halfExtents = toBulletVector(initMsg.box.extents)*0.5*METERS;
	  Eigen::Matrix3f rotation = (Eigen::Matrix3f) Eigen::AngleAxisf(initMsg.box.angle, Eigen::Vector3f::UnitZ());
	  btTransform initTrans(toBulletMatrix(rotation), toBulletVector(initMsg.box.center)*METERS);
	  BoxObject::Ptr sim(new BoxObject(mass, halfExtents, initTrans));
	  env->add(sim);
	  TrackedBox::Ptr tracked_box(new TrackedBox(sim));

	  cv::Mat image = cv::imread("/home/alex/Desktop/image.jpg");
		sim->setTexture(image);

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

TrackedObject::Ptr callInitServiceAndCreateObject(ColorCloudPtr cloud, Environment::Ptr env) {
  bulletsim_msgs::Initialization init;
  pcl::toROSMsg(*cloud, init.request.cloud);
  init.request.cloud.header.frame_id = "/ground";
	
  bool success = ros::service::call(initializationService, init);
  if (success)
  	return toTrackedObject(init.response.objectInit, cloud, env);
  else {
		ROS_ERROR("initialization failed");
		return TrackedObject::Ptr();
  }

}
