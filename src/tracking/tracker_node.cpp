#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <bulletsim_msgs/TrackedObject.h>

#include "clouds/utils_pcl.h"
#include "utils_tracking.h"
#include "utils/logging.h"
#include "utils/vector_alg.h"
#include "visibility.h"
#include "simple_physics_tracker.h"
#include "initialization.h"
#include "simulation/simplescene.h"
#include "config_tracking.h"

using sensor_msgs::PointCloud2;
using sensor_msgs::Image;
using namespace std;

namespace cv {
	typedef Vec<uchar, 3> Vec3b;
}

bool pending = false; // new message received, waiting to be processed
cv::Mat depthImage;
ColorCloudPtr filteredCloud(new ColorCloud()); // filtered cloud in ground frame

CoordinateTransformer* transformer;
tf::TransformListener* listener;

void callback (const sensor_msgs::PointCloud2ConstPtr& cloudMsg,
			        const sensor_msgs::ImageConstPtr& depthMsg) {
  LOG_DEBUG("callback");
  if (transformer == NULL) {
    transformer = new CoordinateTransformer(waitForAndGetTransform(*listener, "/ground",cloudMsg->header.frame_id));
  }
  depthImage = cv_bridge::toCvCopy(depthMsg)->image;
  // toCvShare causes segfault ?!
  pcl::fromROSMsg(*cloudMsg, *filteredCloud);
  pcl::transformPointCloud(*filteredCloud, *filteredCloud, transformer->worldFromCamEigen);
  pending = true;
}

#include "utils/vector_alg.h"
#include "utils/conversions.h"

int main(int argc, char* argv[]) {
  Eigen::internal::setNbThreads(2);
  Parser parser;
  parser.addGroup(TrackingConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  GeneralConfig::scale = 10;
  parser.read(argc, argv);

  ros::init(argc, argv,"tracker_node");
  ros::NodeHandle nh;

  listener = new tf::TransformListener();

  message_filters::Subscriber<PointCloud2> cloudSub(nh, TrackingConfig::filteredCloudTopic,1);
  message_filters::Subscriber<Image> imageSub(nh, TrackingConfig::depthTopic, 1);
  message_filters::TimeSynchronizer<PointCloud2, Image> sync(cloudSub,imageSub,30);
  sync.registerCallback(boost::bind(&callback,_1,_2));

  ros::Publisher objPub = nh.advertise<bulletsim_msgs::TrackedObject>(trackedObjectTopic,10);
	
	
  // wait for first message, then initialize
  while (!pending) {
    ros::spinOnce();
    sleep(.001);
    if (!ros::ok()) throw runtime_error("caught signal while waiting for first message");
  }
  // set up scene
  Scene scene;
  scene.startViewer();


  TrackedObject::Ptr trackedObj = callInitServiceAndCreateObject(scaleCloud(filteredCloud,1/METERS), scene.env);
  if (!trackedObj) throw runtime_error("initialization of object failed.");
  //scene.env->add(trackedObj->m_sim);
  //dynamic_cast<BulletObject*>(trackedObj->getSim())->setTexture();
  //dynamic_cast<BulletSoftObject*>(trackedObj->getSim())->setColor(1,0,0,1);

  // actual tracking algorithm
  //  DepthImageVisibility visInterface(transformer);
    DepthImageVisibility visInterface(transformer);
    SimplePhysicsTracker alg(trackedObj, &visInterface, scene.env);

  TrackedObject::Ptr tracked_rope = trackedObj;
  CapsuleRope* sim = dynamic_cast<CapsuleRope*>(trackedObj->getSim());
  vector<btVector3> nodes = tracked_rope->getPoints();
  ColorCloudPtr cloud = filteredCloud;
  ColorCloudPtr debugCloud(new ColorCloud());

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
		float radius = ((float) TrackingConfig::fixeds)/10.0; //(fixeds in cm)
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		Eigen::Matrix3f node_rot = toEigenMatrix(rotations[j]);
		float node_half_height = half_heights[j];
		vector<vector<float> > R_bins(ang_res*x_res), G_bins(ang_res*x_res), B_bins(ang_res*x_res);
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
					R_bins[xId*ang_res+angId].push_back(cloud->points[pointIdxRadiusSearch[i]].r);
					G_bins[xId*ang_res+angId].push_back(cloud->points[pointIdxRadiusSearch[i]].g);
					B_bins[xId*ang_res+angId].push_back(cloud->points[pointIdxRadiusSearch[i]].b);
				}
				if (xId >= 0 && xId < x_res/2 && j%2==0) {
					debugCloud->push_back(cloud->points[pointIdxRadiusSearch[i]]);
				}
			}
		}
		for (int xId=0; xId<x_res; xId++) {
			for (int angId=0; angId<ang_res; angId++) {
				image.at<cv::Vec3b>(angId,j*x_res+xId)[0] = mean(append(B_bins, xId*ang_res, (xId+1)*ang_res));
				image.at<cv::Vec3b>(angId,j*x_res+xId)[1] = mean(append(G_bins, xId*ang_res, (xId+1)*ang_res));
				image.at<cv::Vec3b>(angId,j*x_res+xId)[2] = mean(append(R_bins, xId*ang_res, (xId+1)*ang_res));
			}
		}
//		for (int xId=0; xId<x_res; xId++) {
//			for (int angId=0; angId<ang_res; angId++) {
//				image.at<cv::Vec3b>(angId,j*x_res+xId)[0] = mean(B_bins[xId*ang_res+angId]);
//				image.at<cv::Vec3b>(angId,j*x_res+xId)[1] = mean(G_bins[xId*ang_res+angId]);
//				image.at<cv::Vec3b>(angId,j*x_res+xId)[2] = mean(R_bins[xId*ang_res+angId]);
//			}
//		}
	}
	cv::imwrite("/home/alex/Desktop/fwd.jpg", image);
	sim->setTexture(image);

	alg.M_obsDebug = toEigenMatrix(debugCloud);


  scene.addVoidKeyCallback('C',boost::bind(toggle, &alg.m_enableCorrPlot));
  scene.addVoidKeyCallback('c',boost::bind(toggle, &alg.m_enableCorrPlot));
  scene.addVoidKeyCallback('E',boost::bind(toggle, &alg.m_enableEstPlot));
  scene.addVoidKeyCallback('e',boost::bind(toggle, &alg.m_enableEstPlot));
  scene.addVoidKeyCallback('O',boost::bind(toggle, &alg.m_enableObsPlot));
  scene.addVoidKeyCallback('o',boost::bind(toggle, &alg.m_enableObsPlot));
  scene.addVoidKeyCallback('L',boost::bind(toggle, &alg.m_enableObsColorPlot));
  scene.addVoidKeyCallback('l',boost::bind(toggle, &alg.m_enableObsColorPlot));
  scene.addVoidKeyCallback('B',boost::bind(toggle, &alg.m_enableObsDebugPlot));
  scene.addVoidKeyCallback('b',boost::bind(toggle, &alg.m_enableObsDebugPlot));
  scene.addVoidKeyCallback('T',boost::bind(toggle, &dynamic_cast<EnvironmentObject*>(trackedObj->getSim())->drawingOn));
  scene.addVoidKeyCallback('t',boost::bind(toggle, &dynamic_cast<EnvironmentObject*>(trackedObj->getSim())->drawingOn));
  scene.addVoidKeyCallback('q',boost::bind(exit, 0));

  while (ros::ok()) {
    alg.updateInput(filteredCloud);
    visInterface.updateInput(depthImage);
    pending = false;
    while (ros::ok() && !pending) {
      alg.doIteration();
      LOG_DEBUG("did iteration");
      scene.viewer.frame();
      ros::spinOnce();
    }
    LOG_DEBUG("publishing");
    //TODO
    //objPub.publish(toTrackedObjectMessage(trackedObj));
  }



}
