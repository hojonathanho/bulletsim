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
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>
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

#include "phasespace/phasespace.h"

using sensor_msgs::PointCloud2;
using sensor_msgs::Image;
using namespace std;
using namespace Eigen;

namespace cv {
	typedef Vec<uchar, 3> Vec3b;
}

vector<cv::Mat> rgb_images;
vector<cv::Mat> depth_images;
vector<CoordinateTransformer*> transformer_images;

int nCameras;

ColorCloudPtr filteredCloud(new ColorCloud()); // filtered cloud in ground frame
CoordinateTransformer* transformer;
bool pending = false; // new message received, waiting to be processed

tf::TransformListener* listener;

Affine3f marker_system_transform;

// TODO this fcn should also mask
void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const vector<sensor_msgs::ImageConstPtr>& image_msgs) {
  pcl::fromROSMsg(*cloud_msg, *filteredCloud);
  pcl::transformPointCloud(*filteredCloud, *filteredCloud, transformer->worldFromCamEigen);

  if (rgb_images.size()!=nCameras) rgb_images.resize(nCameras);
  if (depth_images.size()!=nCameras) depth_images.resize(nCameras);

  assert(image_msgs.size() == 2*nCameras);
  for (int i=0; i<nCameras; i++) {
  	rgb_images[i] = cv_bridge::toCvCopy(image_msgs[i])->image;
  	depth_images[i] = cv_bridge::toCvCopy(image_msgs[i+nCameras])->image;
  }

  pending = true;
}

int main(int argc, char* argv[]) {
  Eigen::internal::setNbThreads(2);

  GeneralConfig::scale = 100;
  BulletConfig::maxSubSteps = 0;
  BulletConfig::gravity = btVector3(0,0,-0.1);

  Parser parser;
  parser.addGroup(TrackingConfig());
  parser.addGroup(PhasespaceConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);

  nCameras = TrackingConfig::cameraTopics.size();

  ros::init(argc, argv,"tracker_node");
  ros::NodeHandle nh;

  listener = new tf::TransformListener();

  transformer = new CoordinateTransformer(waitForAndGetTransform(*listener, "/ground", "kinect1_rgb_optical_frame"));
  for (int i=0; i<nCameras; i++)
  	transformer_images.push_back(new CoordinateTransformer(waitForAndGetTransform(*listener, "/ground", TrackingConfig::cameraTopics[i]+"_rgb_optical_frame")));

  vector<string> image_topics;
	image_topics.push_back("/preprocessor/image");
  image_topics.push_back("/kinect1/depth_registered/image_rect");
  synchronizeAndRegisterCallback("/preprocessor/points", image_topics, nh, callback);

  // wait for first message, then initialize
  while (!pending) {
    ros::spinOnce();
    sleep(.001);
    if (!ros::ok()) throw runtime_error("caught signal while waiting for first message");
  }

  // set up scene
  Scene scene;
  scene.startViewer();

  // Get the filtered cloud that is not downsample: get the full cloud and then mask it with the image.
  sensor_msgs::PointCloud2ConstPtr cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(TrackingConfig::fullCloudTopic, nh);
	ColorCloudPtr first_organized_filtered_cloud(new ColorCloud());
	pcl::fromROSMsg(*cloud_msg, *first_organized_filtered_cloud);
	first_organized_filtered_cloud = maskCloud(first_organized_filtered_cloud, rgb_images[0]);
	pcl::transformPointCloud(*first_organized_filtered_cloud, *first_organized_filtered_cloud, transformer_images[0]->worldFromCamEigen);

  TrackedObject::Ptr trackedObj = callInitServiceAndCreateObject(filteredCloud, first_organized_filtered_cloud, rgb_images[0], transformer_images[0]);
  if (!trackedObj) throw runtime_error("initialization of object failed.");
  trackedObj->init();
  scene.env->add(trackedObj->m_sim);

 	////////////////////////////////////////////////
	Affine3f cameraToPhasespace = waitForRigidBodyTransform(PhasespaceConfig::kinectInfo_filename, 10);
	Affine3f cameraToWorld = toEigenTransform(transformer->worldFromCamUnscaled);
	cameraToWorld.translation() *= METERS;
	Affine3f phasespaceToWorld = cameraToWorld * cameraToPhasespace.inverse();
	marker_system_transform = phasespaceToWorld;

	util::drawAxes(toBulletTransform(cameraToWorld), 0.1*METERS, scene.env);

 	// Contains all the phase space bodies
 	vector<MarkerBody::Ptr> marker_bodies;

 	// Create soft body for cloth
 	cout << "Creating soft body cloth..." << endl;
 	assert(PhasespaceConfig::objLedIds.size()%2 == 0);
 	vector<ledid_t> led_ids_front(PhasespaceConfig::objLedIds.begin(), PhasespaceConfig::objLedIds.begin()+PhasespaceConfig::objLedIds.size()/2.0);
 	vector<ledid_t> led_ids_back(PhasespaceConfig::objLedIds.begin()+PhasespaceConfig::objLedIds.size()/2.0, PhasespaceConfig::objLedIds.end());
 	MarkerSoft::Ptr marker_soft = createMarkerSoftCloth(led_ids_front, led_ids_back, Vector3f(0,0,-0.02*METERS), trackedObj->m_sim, marker_system_transform, scene.env);
 	marker_bodies.push_back(marker_soft);

 	MarkerSystem::Ptr marker_system(new MarkerSystem(marker_bodies));
 	marker_system->setPose(marker_system_transform);

 	marker_system->startUpdateLoopThread();

	vector<PlotAxes::Ptr> plot_axes;
	for (int ind=0; ind<PhasespaceConfig::objLedIds.size(); ind++) {
		plot_axes.push_back(PlotAxes::Ptr (new PlotAxes()));
		scene.env->add(plot_axes[ind]);
	}
 	////////////////////////////////////////////////

  // actual tracking algorithm
	MultiVisibility::Ptr visInterface(new MultiVisibility());
	for (int i=0; i<nCameras; i++) {
		if (trackedObj->m_type == "rope") // Don't do self-occlusion if the trackedObj is a rope
			visInterface->addVisibility(DepthImageVisibility::Ptr(new DepthImageVisibility(transformer_images[i])));
		else
			visInterface->addVisibility(AllOcclusionsVisibility::Ptr(new AllOcclusionsVisibility(scene.env->bullet->dynamicsWorld, transformer_images[i])));
	}

	TrackedObjectFeatureExtractor::Ptr objectFeatures(new TrackedObjectFeatureExtractor(trackedObj));
	CloudFeatureExtractor::Ptr cloudFeatures(new CloudFeatureExtractor());
	PhysicsTracker::Ptr alg(new PhysicsTracker(objectFeatures, cloudFeatures, visInterface));
	PhysicsTrackerVisualizer::Ptr trackingVisualizer(new PhysicsTrackerVisualizer(&scene, alg));

	bool applyEvidence = true;
  scene.addVoidKeyCallback('a',boost::bind(toggle, &applyEvidence));
  scene.addVoidKeyCallback('=',boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), 0.1f));
  scene.addVoidKeyCallback('-',boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), -0.1f));
  bool exit_loop = false;
  scene.addVoidKeyCallback('q',boost::bind(toggle, &exit_loop));

  while (!exit_loop && ros::ok()) {
  	//Update the inputs of the featureExtractors and visibilities (if they have any inputs)
  	cloudFeatures->updateInputs(filteredCloud, rgb_images[0], transformer_images[0]);
  	for (int i=0; i<nCameras; i++)
    	visInterface->visibilities[i]->updateInput(depth_images[i]);
    pending = false;
    while (ros::ok() && !pending) {
    	//Do iteration
      alg->updateFeatures();
      alg->expectationStep();
      alg->maximizationStep(applyEvidence);

      scene.env->step(.03,2,.015);

  		for (int i=0; i<marker_bodies.size(); i++)
  			marker_bodies[i]->plot();
      for (int ind=0; ind<plot_axes.size(); ind++)
  			plot_axes[ind]->setup(toBulletTransform(marker_soft->getSimTransform(ind)), 0.02*METERS);

      trackingVisualizer->update();

      scene.draw();
      ros::spinOnce();
    }
 	}
  marker_system->stopUpdateLoopThread();
}
