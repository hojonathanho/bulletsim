#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <bulletsim_msgs/TrackedObject.h>

#include "clouds/utils_pcl.h"
#include "utils_tracking.h"
#include "utils/logging.h"
#include "visibility.h"
#include "simple_physics_tracker.h"
#include "initialization.h"
#include "simulation/simplescene.h"
#include "config_tracking.h"

using sensor_msgs::PointCloud2;
using sensor_msgs::Image;
using namespace std;

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
  scene.env->add(trackedObj->m_sim);
	// actual tracking algorithm
//  DepthImageVisibility visInterface(transformer);
  DepthImageVisibility visInterface(transformer);
  SimplePhysicsTracker alg(trackedObj, &visInterface, scene.env);

  scene.addVoidKeyCallback('C',boost::bind(toggle, &alg.m_enableCorrPlot));
  scene.addVoidKeyCallback('c',boost::bind(toggle, &alg.m_enableCorrPlot));
  scene.addVoidKeyCallback('E',boost::bind(toggle, &alg.m_enableEstPlot));
  scene.addVoidKeyCallback('e',boost::bind(toggle, &alg.m_enableEstPlot));
  scene.addVoidKeyCallback('O',boost::bind(toggle, &alg.m_enableObsPlot));
  scene.addVoidKeyCallback('o',boost::bind(toggle, &alg.m_enableObsPlot));
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
