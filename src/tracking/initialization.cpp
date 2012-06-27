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
#include "utils/vector_alg.h"

using namespace std;

TrackedObject::Ptr toTrackedObject(const bulletsim_msgs::ObjectInit& initMsg, ColorCloudPtr cloud, cv::Mat image, CoordinateTransformer* transformer, Environment::Ptr env) {
  if (initMsg.type == "rope") {
	  vector<btVector3> nodes = toBulletVectors(initMsg.rope.nodes);
//		//downsample nodes
//		vector<btVector3> nodes;
//		for (int i=0; i<nodes_o.size(); i+=3)
//			nodes.push_back(nodes_o[i]);
	  BOOST_FOREACH(btVector3& node, nodes) node += btVector3(0,0,.01);

	  CapsuleRope::Ptr sim(new CapsuleRope(scaleVecs(nodes,METERS), initMsg.rope.radius*METERS));
	  env->add(sim);
	  TrackedObject::Ptr tracked_rope(new TrackedRope(sim));

	  int x_res = 3;
		int ang_res = 1;
		cv::Mat image(ang_res, nodes.size()*x_res, CV_8UC3);
		vector<btMatrix3x3> rotations = sim->getRotations();
		vector<float> half_heights = sim->getHalfHeights();
		for (int j=0; j<nodes.size(); j++) {
			pcl::KdTreeFLANN<ColorPoint> kdtree;
			kdtree.setInputCloud(cloud);
			ColorPoint searchPoint;
			searchPoint.x = nodes[j].x();
			searchPoint.y = nodes[j].y();
			searchPoint.z = nodes[j].z();
			// Neighbors within radius search
	//		float radius = ((float) TrackingConfig::fixeds)/10.0; //(fixeds in cm)
			float radius = ((float) 3)/10.0; //(fixeds in cm)
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			Eigen::Matrix3f node_rot = toEigenMatrix(rotations[j]);
			float node_half_height = half_heights[j];
			vector<vector<float> > B_bins(ang_res*x_res), G_bins(ang_res*x_res), R_bins(ang_res*x_res);
			if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
				for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++) {
					Eigen::Vector3f alignedPoint = node_rot * (toEigenVector(cloud->points[pointIdxRadiusSearch[i]]) - toEigenVector(searchPoint));
					int xId = (int) floor( ((float) x_res) * (1.0 + alignedPoint(0)/node_half_height) * 0.5 );
					float angle = atan2(alignedPoint(2), alignedPoint(1))*180.0/M_PI;
					if (angle >= 90) angle-=90;
					else angle+=270;
					angle = 360-angle;
					//if (angle<0) angle+=360.0;
					int angId = (int) floor( ((float) ang_res) * angle/360.0 );
					assert(angId >= 0 && angId < ang_res);
					if (xId >= 0 && xId < x_res) {
						B_bins[xId*ang_res+angId].push_back(cloud->points[pointIdxRadiusSearch[i]].b);
						G_bins[xId*ang_res+angId].push_back(cloud->points[pointIdxRadiusSearch[i]].g);
						R_bins[xId*ang_res+angId].push_back(cloud->points[pointIdxRadiusSearch[i]].r);
					}
//					if (xId >= 0 && xId < x_res/2 && j%2==0) {
//						debugCloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
//					}
				}
			}
	//		for (int xId=0; xId<x_res; xId++) {
	//			for (int angId=0; angId<ang_res; angId++) {
	//				image.at<cv::Vec3b>(angId,j*x_res+xId)[0] = mean(append(B_bins, xId*ang_res, (xId+1)*ang_res));
	//				image.at<cv::Vec3b>(angId,j*x_res+xId)[1] = mean(append(G_bins, xId*ang_res, (xId+1)*ang_res));
	//				image.at<cv::Vec3b>(angId,j*x_res+xId)[2] = mean(append(R_bins, xId*ang_res, (xId+1)*ang_res));
	//			}
	//		}
			for (int xId=0; xId<x_res; xId++) {
				for (int angId=0; angId<ang_res; angId++) {
					image.at<cv::Vec3b>(angId,j*x_res+xId)[0] = mean(B_bins[xId*ang_res+angId]);
					image.at<cv::Vec3b>(angId,j*x_res+xId)[1] = mean(G_bins[xId*ang_res+angId]);
					image.at<cv::Vec3b>(angId,j*x_res+xId)[2] = mean(R_bins[xId*ang_res+angId]);
				}
			}
		}
		//cv::imwrite("/home/alex/Desktop/fwd.jpg", image);
		sim->setTexture(image);

	  return tracked_rope;
  }
  else if (initMsg.type == "towel_corners") {
	  const vector<geometry_msgs::Point32>& points = initMsg.towel_corners.polygon.points;
	  vector<btVector3> corners = scaleVecs(toBulletVectors(points),METERS);

	  BulletSoftObject::Ptr sim = makeTowel(corners, TrackingConfig::res_x, TrackingConfig::res_y, env->bullet->softBodyWorldInfo);
	  cv::Mat tex_image = makeTowelTexture(corners, image, transformer);
		sim->setTexture(tex_image);
	  TrackedObject::Ptr tracked_towel(new TrackedTowel(sim, TrackingConfig::res_x, TrackingConfig::res_y));

	  env->add(sim);

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

TrackedObject::Ptr callInitServiceAndCreateObject(ColorCloudPtr cloud, cv::Mat image, CoordinateTransformer* transformer, Environment::Ptr env) {
  bulletsim_msgs::Initialization init;
  pcl::toROSMsg(*cloud, init.request.cloud);
  init.request.cloud.header.frame_id = "/ground";
	
  bool success = ros::service::call(initializationService, init);
  if (success)
  	return toTrackedObject(init.response.objectInit, scaleCloud(cloud,METERS), image, transformer, env);
  else {
		ROS_ERROR("initialization failed");
		return TrackedObject::Ptr();
  }

}
