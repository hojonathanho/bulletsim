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
#include "robots/ros2rave.h"
#include "robots/grabbing.h"
#include "clouds/utils_pcl.h"
#include "utils/logging.h"
#include "utils_tracking.h"
#include "visibility.h"
#include "simple_physics_tracker.h"
#include "initialization.h"
#include "simulation/simplescene.h"
#include "config_tracking.h"

#include "grab_detection.h"

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

GrabDetector* leftGrabDetector;

void callback (const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
	tf::StampedTransform st;
	try {
		listener->lookupTransform("base_footprint", cloudMsg->header.frame_id, ros::Time(0), st);
	}
	catch (tf::TransformException e) {
		printf("tf error: %s\n", e.what());
		return;
	}
  btTransform wfc = st.asBt();
  transformer->reset(wfc);

  inputCloudFrame = cloudMsg->header.frame_id;
  pcl::fromROSMsg(*cloudMsg, *filteredCloud);
  pcl::transformPointCloud(*filteredCloud, *filteredCloud, transformer->worldFromCamEigen);
  pending = true;
}

void jointCallback(const sensor_msgs::JointState& msg) {
  lastJointMsg = msg;
  if (leftGrabDetector != NULL) leftGrabDetector->update(msg);
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

  transformer = new CoordinateTransformer();
  listener = new tf::TransformListener;


  ros::Subscriber cloudSub = nh.subscribe(TrackingConfig::filteredCloudTopic,1, callback);
  ros::Subscriber jointSub = nh.subscribe("/joint_states", 1, jointCallback);
  ros::Publisher objPub = nh.advertise<bulletsim_msgs::TrackedObject>(trackedObjectTopic,10);
	
  // wait for first message, then initialize
  while (!pending || lastJointMsg.position.size()==0) {
    ros::spinOnce();
    sleep(.001);
    if (!ros::ok()) throw runtime_error("caught signal while waiting for first message");
  }  
  
  // set up scene
  Scene scene;
  PR2Manager pr2m(scene);
  scene.startViewer();  
  setupROSRave(pr2m.pr2->robot, lastJointMsg);

  TrackedObject::Ptr trackedObj = callInitServiceAndCreateObject(scaleCloud(filteredCloud,1/METERS), scene.env);
  if (!trackedObj) throw runtime_error("initialization of object failed.");
  scene.env->add(trackedObj->m_sim);

//  SoftMonitorForGrabbing lMonitor(pr2m.pr2, pr2m.pr2Left, true);
//  SoftMonitorForGrabbing rMonitor(pr2m.pr2, pr2m.pr2Right, false);

  PR2SoftBodyGripper leftSoftGripper(pr2m.pr2, pr2m.pr2Left->manip, true);


  TrackedTowel::Ptr maybeTowel = boost::dynamic_pointer_cast<TrackedTowel>(trackedObj);
  BulletSoftObject::Ptr towelSim;
  bool trackingTowel = !!maybeTowel;
  if (trackingTowel) {
	  towelSim.reset(maybeTowel->getSim());
//	  lMonitor.setTarget(towelSim);
//	  rMonitor.setTarget(towelSim);
	  leftSoftGripper.setTarget(towelSim);
	  leftGrabDetector = new GrabDetector(GrabDetector::LEFT,
			  boost::bind(&PR2SoftBodyGripper::grab, leftSoftGripper),
			  boost::bind(&PR2SoftBodyGripper::releaseAllAnchors, leftSoftGripper));
  }


//  BulletRaycastVisibility visInterface(scene.env->bullet->dynamicsWorld, transformer);
  EverythingIsVisible visInterface;
  SimplePhysicsTracker alg(trackedObj, &visInterface, scene.env);

  Load(scene.env, scene.rave, "/home/joschu/python/lfd/data/table.xml");
  
  scene.addVoidKeyCallback('c',boost::bind(toggle, &alg.m_enableCorrPlot));
  scene.addVoidKeyCallback('e',boost::bind(toggle, &alg.m_enableEstPlot));
  scene.addVoidKeyCallback('o',boost::bind(toggle, &alg.m_enableObsPlot));
  scene.addVoidKeyCallback('t',boost::bind(toggle, &dynamic_cast<EnvironmentObject*>(trackedObj->getSim())->drawingOn));
  scene.addVoidKeyCallback('q',boost::bind(exit, 0));
  scene.loopState.debugDraw = true;

  while (ros::ok()) {
    alg.updateInput(filteredCloud);
    pending = false;

    while (ros::ok() && !pending) {
      ValuesInds vi = getValuesInds(lastJointMsg.position);
      pr2m.pr2->setDOFValues(vi.second, vi.first);
      alg.doIteration();
      scene.draw();
      ros::spinOnce();
    }
    LOG_DEBUG("publishing");
    //TODO
    //objPub.publish(toTrackedObjectMessage(trackedObj));
  }




}
