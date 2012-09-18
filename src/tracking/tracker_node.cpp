#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <bulletsim_msgs/TrackedObject.h>
#include "clouds/utils_ros.h"

#include "clouds/utils_pcl.h"
#include "utils_tracking.h"
#include "utils/logging.h"
#include "utils/utils_vector.h"
#include "visibility.h"
#include "physics_tracker.h"
#include "feature_extractor.h"
#include "initialization.h"
#include "simulation/simplescene.h"
#include "config_tracking.h"
#include "utils/conversions.h"
#include "clouds/cloud_ops.h"
#include "simulation/util.h"
#include "clouds/utils_cv.h"

using sensor_msgs::PointCloud2;
using sensor_msgs::Image;
using namespace std;
using namespace Eigen;

namespace cv {
	typedef Vec<uchar, 3> Vec3b;
}

int nCameras;

vector<cv::Mat> rgb_images;
vector<cv::Mat> mask_images;
vector<cv::Mat> depth_images;
vector<CoordinateTransformer*> transformers;

ColorCloudPtr filteredCloud(new ColorCloud()); // filtered cloud in ground frame
bool pending = false; // new message received, waiting to be processed

tf::TransformListener* listener;

void callback(const vector<sensor_msgs::PointCloud2ConstPtr>& cloud_msg, const vector<sensor_msgs::ImageConstPtr>& image_msgs) {
  if (rgb_images.size()!=nCameras) rgb_images.resize(nCameras);
  if (mask_images.size()!=nCameras) mask_images.resize(nCameras);
  if (depth_images.size()!=nCameras) depth_images.resize(nCameras);

  assert(image_msgs.size() == 2*nCameras);
  for (int i=0; i<nCameras; i++) {
  	// merge all the clouds progressively
  	ColorCloudPtr cloud(new ColorCloud);
		pcl::fromROSMsg(*cloud_msg[i], *cloud);
		pcl::transformPointCloud(*cloud, *cloud, transformers[i]->worldFromCamEigen);

		if (i==0) *filteredCloud = *cloud;
  	else *filteredCloud = *filteredCloud + *cloud;

	extractImageAndMask(cv_bridge::toCvCopy(image_msgs[2*i])->image, rgb_images[i], mask_images[i]);
  	depth_images[i] = cv_bridge::toCvCopy(image_msgs[2*i+1])->image;
  }
  filteredCloud = downsampleCloud(filteredCloud, TrackingConfig::downsample*METERS);

  pending = true;
}

int main(int argc, char* argv[]) {
  Eigen::internal::setNbThreads(2);

  GeneralConfig::scale = 100;
  BulletConfig::maxSubSteps = 0;
  BulletConfig::gravity = btVector3(0,0,-0.1);

  Parser parser;
  parser.addGroup(TrackingConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);

  nCameras = TrackingConfig::cameraTopics.size();

  ros::init(argc, argv,"tracker_node");
  ros::NodeHandle nh;

  listener = new tf::TransformListener();

  for (int i=0; i<nCameras; i++)
  	transformers.push_back(new CoordinateTransformer(waitForAndGetTransform(*listener, "/ground", TrackingConfig::cameraTopics[i]+"_rgb_optical_frame")));

	vector<string> cloud_topics;
	vector<string> image_topics;
  for (int i=0; i<nCameras; i++) {
		cloud_topics.push_back("/preprocessor" + TrackingConfig::cameraTopics[i] + "/points");
		image_topics.push_back("/preprocessor" + TrackingConfig::cameraTopics[i] + "/image");
		image_topics.push_back("/preprocessor" + TrackingConfig::cameraTopics[i] + "/depth");
  }
	synchronizeAndRegisterCallback(cloud_topics, image_topics, nh, callback);

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

  util::setGlobalEnv(scene.env);

	TrackedObject::Ptr trackedObj = callInitServiceAndCreateObject(filteredCloud, rgb_images[0], transformers[0]);
  if (!trackedObj) throw runtime_error("initialization of object failed.");
  trackedObj->init();
  scene.env->add(trackedObj->m_sim);

 	// actual tracking algorithm
	MultiVisibility::Ptr visInterface(new MultiVisibility());
	for (int i=0; i<nCameras; i++) {
		if (trackedObj->m_type == "rope") // Don't do self-occlusion if the trackedObj is a rope
			visInterface->addVisibility(DepthImageVisibility::Ptr(new DepthImageVisibility(transformers[i])));
		else
			visInterface->addVisibility(AllOcclusionsVisibility::Ptr(new AllOcclusionsVisibility(scene.env->bullet->dynamicsWorld, transformers[i])));
	}

	TrackedObjectFeatureExtractor::Ptr objectFeatures(new TrackedObjectFeatureExtractor(trackedObj));
	CloudFeatureExtractor::Ptr cloudFeatures(new CloudFeatureExtractor());
	PhysicsTracker::Ptr alg(new PhysicsTracker(objectFeatures, cloudFeatures, visInterface));
	PhysicsTrackerVisualizer::Ptr trackingVisualizer(new PhysicsTrackerVisualizer(&scene, alg));

	bool applyEvidence = true;
  scene.addVoidKeyCallback('a',boost::bind(toggle, &applyEvidence), "apply evidence");
  scene.addVoidKeyCallback('=',boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), 0.1f), "increase opacity");
  scene.addVoidKeyCallback('-',boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), -0.1f), "decrease opacity");
  bool exit_loop = false;
  scene.addVoidKeyCallback('q',boost::bind(toggle, &exit_loop), "exit");

  while (!exit_loop && ros::ok()) {
  	//Update the inputs of the featureExtractors and visibilities (if they have any inputs)
  	cloudFeatures->updateInputs(filteredCloud, rgb_images[0], transformers[0]);
  	for (int i=0; i<nCameras; i++)
    	visInterface->visibilities[i]->updateInput(depth_images[i]);
    pending = false;
    while (ros::ok() && !pending) {
    	//Do iteration
      alg->updateFeatures();
      alg->expectationStep();
      alg->maximizationStep(applyEvidence);

      scene.env->step(.03,2,.015);

      trackingVisualizer->update();

      scene.draw();
      ros::spinOnce();
    }
    objPub.publish(toTrackedObjectMessage(trackedObj));
 	}
}
