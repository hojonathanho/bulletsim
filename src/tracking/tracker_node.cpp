#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <bulletsim_msgs/TrackedObject.h>

#include "clouds/utils_pcl.h"
#include "utils_tracking.h"
#include "utils/logging.h"
#include "utils/utils_vector.h"
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

cv::Mat depthImage;
cv::Mat rgbImage;
vector<cv::Mat> depthImages;
vector<cv::Mat> rgbImages;
vector<CoordinateTransformer*> transformer_images;
vector<bool> pending_images;

ColorCloudPtr filteredCloud(new ColorCloud()); // filtered cloud in ground frame
CoordinateTransformer* transformer;
bool pending = false; // new message received, waiting to be processed

tf::TransformListener* listener;

void cloudAndImagesCallback (const sensor_msgs::PointCloud2ConstPtr& cloudMsg,
			        const sensor_msgs::ImageConstPtr& depthImageMsg,
			        const sensor_msgs::ImageConstPtr& rgbImageMsg) {
  LOG_DEBUG("callback");
  if (transformer == NULL) {
    transformer = new CoordinateTransformer(waitForAndGetTransform(*listener, "/ground",cloudMsg->header.frame_id));
  }
  depthImage = cv_bridge::toCvCopy(depthImageMsg)->image;
  rgbImage = cv_bridge::toCvCopy(rgbImageMsg)->image;
  // toCvShare causes segfault ?!
  pcl::fromROSMsg(*cloudMsg, *filteredCloud);
  pcl::transformPointCloud(*filteredCloud, *filteredCloud, transformer->worldFromCamEigen);

  pending = true;
  //cout << "pending All " << pending << endl;
}

void cloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
  if (transformer == NULL) {
    transformer = new CoordinateTransformer(waitForAndGetTransform(*listener, "/ground",cloudMsg->header.frame_id));
  }

  pcl::fromROSMsg(*cloudMsg, *filteredCloud);
  pcl::transformPointCloud(*filteredCloud, *filteredCloud, transformer->worldFromCamEigen);

  pending = true;
  //cout << "pending " << pending << endl;
}

void imagesCallback (const sensor_msgs::ImageConstPtr& depthImageMsg,
										 const sensor_msgs::ImageConstPtr& rgbImageMsg,
										 int i) {
	if (transformer_images[i] == NULL) {
		transformer_images[i] = new CoordinateTransformer(waitForAndGetTransform(*listener, "/ground", TrackingConfig::cameraTopics[0]+"_rgb_optical_frame"));
  }

  depthImages[i] = cv_bridge::toCvCopy(depthImageMsg)->image;
  rgbImages[i] = cv_bridge::toCvCopy(rgbImageMsg)->image;

  pending_images[i] = true;
  //cout << "pending_images[" << i << "] " << pending_images[i] << endl;
}

void adjustTransparency(TrackedObject::Ptr trackedObj, float increment) {
	if (trackedObj->m_type == "rope")
		dynamic_cast<CapsuleRope*>(trackedObj->getSim())->adjustTransparency(increment);
	if (trackedObj->m_type == "towel")
		dynamic_cast<BulletSoftObject*>(trackedObj->getSim())->adjustTransparency(increment);
	if (trackedObj->m_type == "box")
		dynamic_cast<BulletObject*>(trackedObj->getSim())->adjustTransparency(increment);
}

int main(int argc, char* argv[]) {
  Eigen::internal::setNbThreads(2);
  Parser parser;
  parser.addGroup(TrackingConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  GeneralConfig::scale = 10;
  parser.read(argc, argv);

  int nCameras = TrackingConfig::cameraTopics.size();
  transformer_images.assign(nCameras, NULL);
  pending_images.assign(nCameras, false);

  ros::init(argc, argv,"tracker_node");
  ros::NodeHandle nh;

  listener = new tf::TransformListener();

//  message_filters::Subscriber<PointCloud2> cloudSub(nh, TrackingConfig::filteredCloudTopic,1);
//  message_filters::Subscriber<Image> depthImageSub(nh, TrackingConfig::depthTopic, 1);
//  message_filters::Subscriber<Image> rgbImageSub(nh, TrackingConfig::rgbTopic, 1);
//	typedef message_filters::sync_policies::ApproximateTime<PointCloud2, Image, Image> ApproxSyncPolicyAll;
//	message_filters::Synchronizer<ApproxSyncPolicyAll> syncAll(ApproxSyncPolicyAll(30), cloudSub, depthImageSub, rgbImageSub);
//  syncAll.registerCallback(boost::bind(&cloudAndImagesCallback,_1,_2,_3));

  ros::Subscriber cloudSub = nh.subscribe(TrackingConfig::filteredCloudTopic, 1, &cloudCallback);

  depthImages.resize(nCameras);
  rgbImages.resize(nCameras);
	vector<message_filters::Subscriber<Image>*> depthImagesSub(nCameras, NULL);
	vector<message_filters::Subscriber<Image>*> rgbImagesSub(nCameras, NULL);
	typedef message_filters::sync_policies::ApproximateTime<Image, Image> ApproxSyncPolicy;
	vector<message_filters::Synchronizer<ApproxSyncPolicy>*> imagesSync(nCameras, NULL);
	for (int i=0; i<nCameras; i++) {
		depthImagesSub[i] = new message_filters::Subscriber<Image>(nh, TrackingConfig::cameraTopics[i] + "/depth_registered/image_rect", 1);
		rgbImagesSub[i] = new message_filters::Subscriber<Image>(nh, TrackingConfig::cameraTopics[i] + "/rgb/image_rect_color", 1);
		imagesSync[i] = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(30), *depthImagesSub[i], *rgbImagesSub[i]);
		imagesSync[i]->registerCallback(boost::bind(&imagesCallback,_1,_2,i));
	}

  ros::Publisher objPub = nh.advertise<bulletsim_msgs::TrackedObject>(trackedObjectTopic,10);

  // wait for first message, then initialize
  while (!(pending && cwiseAnd(pending_images))) {
    ros::spinOnce();
    sleep(.001);
    if (!ros::ok()) throw runtime_error("caught signal while waiting for first message");
  }
  // set up scene
  Scene scene;
  scene.startViewer();

  TrackedObject::Ptr trackedObj = callInitServiceAndCreateObject(scaleCloud(filteredCloud,1/METERS), rgbImages[0], transformer_images[0], scene.env);

  //TrackedObject::Ptr trackedObj = callInitServiceAndCreateObject(scaleCloud(filteredCloud,1/METERS), rgbImage, transformer, scene.env);
  if (!trackedObj) throw runtime_error("initialization of object failed.");
  //scene.env->add(trackedObj->m_sim);

  // actual tracking algorithm
	//DepthImageVisibility visInterface(transformer);
	//OSGVisibility visInterface(transformer);
	//BulletVisibility visInterface(transformer);
	MultiVisibility visInterface;
	for (int i=0; i<nCameras; i++) {
		visInterface.addVisibility(new OSGVisibility(transformer_images[i]));
	}
	SimplePhysicsTracker alg(trackedObj, &visInterface, scene.env);

  scene.addVoidKeyCallback('c',boost::bind(toggle, &alg.m_enableCorrPlot));
  scene.addVoidKeyCallback('C',boost::bind(toggle, &alg.m_enableCorrPlot));
  scene.addVoidKeyCallback('e',boost::bind(toggle, &alg.m_enableEstPlot));
  scene.addVoidKeyCallback('E',boost::bind(toggle, &alg.m_enableEstTransPlot));
  scene.addVoidKeyCallback('o',boost::bind(toggle, &alg.m_enableObsPlot));
  scene.addVoidKeyCallback('O',boost::bind(toggle, &alg.m_enableObsTransPlot));
  scene.addVoidKeyCallback('i',boost::bind(toggle, &alg.m_enableObsInlierPlot));
  scene.addVoidKeyCallback('I',boost::bind(toggle, &alg.m_enableObsInlierPlot));
  scene.addVoidKeyCallback('b',boost::bind(toggle, &alg.m_enableDebugPlot));
  scene.addVoidKeyCallback('B',boost::bind(toggle, &alg.m_enableDebugPlot));
  scene.addVoidKeyCallback('a',boost::bind(toggle, &alg.m_applyEvidence));
  scene.addVoidKeyCallback('=',boost::bind(adjustTransparency, trackedObj, 0.1f));
  scene.addVoidKeyCallback('-',boost::bind(adjustTransparency, trackedObj, -0.1f));
  scene.addVoidKeyCallback('q',boost::bind(exit, 0));

  while (ros::ok()) {
    alg.updateInput(filteredCloud);
    //visInterface.updateInput(depthImage);
    pending = false;
    while (ros::ok() && !pending) {
      alg.doIteration();
    	scene.env->step(.03,2,.015);
      LOG_DEBUG("did iteration");
      scene.viewer.frame();
      ros::spinOnce();
    }
    LOG_DEBUG("publishing");
    //TODO
    //objPub.publish(toTrackedObjectMessage(trackedObj));
  }



}
