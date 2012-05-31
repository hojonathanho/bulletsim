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
#include <sensor_msgs/JointState.h>

#include "robots/pr2.h"
#include "clouds/utils_pcl.h"
#include "utils_tracking.h"
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
// XXX supposedly global initialization is bad

CoordinateTransformer* transformer;
tf::TransformListener* listener;
string inputCloudFrame;

sensor_msgs::JointState lastJointMsg;

void callback (const sensor_msgs::PointCloud2ConstPtr& cloudMsg,
			        const sensor_msgs::ImageConstPtr& depthMsg) {
  if (transformer == NULL) { // first time
    inputCloudFrame = cloudMsg->header.frame_id;
    transformer = new CoordinateTransformer(waitForAndGetTransform(*listener, inputCloudFrame,cloudMsg->header.frame_id));
  }
  else {
    transformer->reset(waitForAndGetTransform(*listener, inputCloudFrame, cloudMSg->header.frame_id));
  }
  depthImage = cv_bridge::toCvShare(depthMsg)->image;
  pcl::fromROSMsg(*cloudMsg, *filteredCloud);
  pcl::transformPointCloud(*filteredCloud, *filteredCloud, transformer->worldFromCamEigen);
  pending = true;
}

void jointCallback(const sensor_msgs::JointState& msg) {
  lastJointMsg = 
}

int main(int argc, char* argv[]) {

  ros::init(argc, argv,"tracker_node");
  ros::NodeHandle nh;

  tf::TransformListener* listener = new tf::TransformListener;

  message_filters::Subscriber<PointCloud2> cloudSub(nh, TrackingConfig::filteredCloudTopic,1);
  message_filters::Subscriber<Image> imageSub(nh, TrackingConfig::depthTopic, 1);
  message_filters::TimeSynchronizer<PointCloud2, Image> sync(cloudSub,imageSub,10);
  sync.registerCallback(boost::bind(&callback,_1,_2));

  ros::Publisher objPub = nh.advertise<bulletsim_msgs::TrackedObject>(trackedObjectTopic,10);
	
  // wait for first message, then initialize
  while (!pending) {
    ros::spinOnce();
    sleep(.001);
  }  
  TrackedObject::Ptr trackedObj = callInitServiceAndCreateObject(filteredCloud);
  
  // set up scene
  Scene scene;
  PR2Manager pr2m(scene);
  scene.startViewer();  
  scene.env->add(trackedObj->m_sim);
	
	// actual tracking algorithm
  DepthImageVisibility visInterface(transformer);
  SimplePhysicsTracker alg(trackedObj, &visInterface);
  
  while (ros::ok()) {
    ros::spinOnce();
    alg.updateInput(filteredCloud);
    visInterface.updateInput(depthImage);
    while (ros::ok() && !pending) {
      alg.doIteration();
    }
    objPub.publish(toTrackedObjectMessage(trackedObj));
  }



}
