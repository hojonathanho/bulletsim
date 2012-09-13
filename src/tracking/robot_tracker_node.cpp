#include <sensor_msgs/JointState.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <bulletsim_msgs/TrackedObject.h>
#include "clouds/utils_ros.h"
#include "utils/logging.h"
#include "visibility.h"
#include "physics_tracker.h"
#include "initialization.h"
#include "simulation/simplescene.h"
#include "config_tracking.h"
#include "robots/pr2.h"
#include "robots/ros2rave.h"
#include "robots/grabbing.h"
#include "grab_detection.h"
#include "simulation/bullet_io.h"
#include "clouds/ros_robot.h"
#include <std_srvs/Empty.h>
#include "demo_recorder.h"

boost::shared_ptr<CoordinateTransformer> transformer;
boost::shared_ptr<tf::TransformListener> listener;

TrackedObject::Ptr trackedObj;
TrackedObjectFeatureExtractor::Ptr objectFeatures;
Environment::Ptr env;

bool pending = false;
ColorCloudPtr filteredCloud;
void cloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
  btTransform wfc;
  if (!lookupLatestTransform(wfc, "base_footprint", cloudMsg->header.frame_id, *listener)) return;
  transformer->reset(wfc);
  filteredCloud = fromROSMsg1(*cloudMsg);
  pcl::transformPointCloud(*filteredCloud, *filteredCloud, transformer->worldFromCamEigen);
  pending = true;
}

void initializeTrackedObject() {
  trackedObj = callInitServiceAndCreateObject(scaleCloud(filteredCloud,1/METERS), cv::Mat(), 0);
  if (!trackedObj) throw runtime_error("initialization of object failed.");
  LOG_INFO("created an object of type " << trackedObj->m_type);
  trackedObj->init();
  env->add(trackedObj->m_sim);
}



bool reinitialize(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp) {
  if (trackedObj) {
    env->remove(trackedObj->m_sim);
  }
  initializeTrackedObject();
  objectFeatures->setObj(trackedObj);
  return true;
}

int main(int argc, char* argv[]) {
  Eigen::internal::setNbThreads(2);
  Parser parser;
  parser.addGroup(TrackingConfig());
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(RecordingConfig());
  GeneralConfig::scale = 10;
  parser.read(argc, argv);

  ros::init(argc, argv,"tracker_node");
  ros::NodeHandle nh;

  transformer.reset(new CoordinateTransformer());
  listener.reset(new tf::TransformListener());

  ros::Subscriber cloudSub = nh.subscribe(TrackingConfig::filteredCloudTopic, 1, &cloudCallback);
  ros::Publisher objPub = nh.advertise<bulletsim_msgs::TrackedObject>(trackedObjectTopic,10);
  ros::ServiceServer resetSrv = nh.advertiseService("/tracker/reinitialize",   reinitialize);

  Scene scene;
  env = scene.env;
  PR2Manager pr2m(scene);
  RobotSync sync(nh, pr2m.pr2);


  DemoRecorder::Ptr demoRecord;
  if (RecordingConfig::record) {
    demoRecord.reset(new DemoRecorder(nh, "/camera/rgb/image_rect_color", scene.viewer));
  }
  boost::thread recordThread(boost::bind(&DemoRecorder::frameLoop, demoRecord.get(), 20));


  CoordinateTransformer cam_transformer;
  btTransform baseFromCam;
  while (true) {
    if (lookupLatestTransform(baseFromCam, "base_footprint", "camera_rgb_optical_frame", *listener)) {
      cam_transformer.reset(baseFromCam);
      break;
    }
    else LOG_INFO("waiting for tf to work");
  }


  Load(scene.env, scene.rave, EXPAND(BULLETSIM_DATA_DIR)"/xml/table.xml");
  RaveObject::Ptr table = getObjectByName(scene.env, scene.rave, "table");
  table->setColor(0,.8,0,.5);
  pr2m.pr2->setColor(1,1,1,.5);
//
//  MonitorForGrabbing lMonitor(pr2m.pr2Left, scene.env->bullet->dynamicsWorld);
//  MonitorForGrabbing rMonitor(pr2m.pr2Right, scene.env->bullet->dynamicsWorld);
//  GrabDetector* leftGrabDetector = new GrabDetector(GrabDetector::LEFT,
//          boost::bind(&MonitorForGrabbing::grab, &lMonitor),
//          boost::bind(&MonitorForGrabbing::release, &lMonitor));
//  GrabDetector* rightGrabDetector = new GrabDetector(GrabDetector::RIGHT,
//          boost::bind(&MonitorForGrabbing::grab, &rMonitor),
//          boost::bind(&MonitorForGrabbing::release, &rMonitor));

  scene.startViewer();

  while (ros::ok()) {
    ros::spinOnce();
    if (filteredCloud && sync.m_lastMsg.position.size()>0) break;
    sleep(.001);
  }
  if (!ros::ok()) throw runtime_error("caught signal while waiting for first message");


  initializeTrackedObject();

//  CapsuleRope* maybeRope = dynamic_cast<CapsuleRope*>(trackedObj->m_sim.get());
//  if (maybeRope) {
//    printf("setting children\n");
//    lMonitor.setBodies(maybeRope->getChildren());
//    rMonitor.setBodies(maybeRope->getChildren());
//  }

  VisibilityInterface::Ptr visInterface(new BulletRaycastVisibility(scene.env->bullet->dynamicsWorld, &cam_transformer));

  objectFeatures.reset(new TrackedObjectFeatureExtractor(trackedObj));
  CloudFeatureExtractor::Ptr cloudFeatures(new CloudFeatureExtractor());
  PhysicsTracker::Ptr alg(new PhysicsTracker(objectFeatures, cloudFeatures, visInterface));
  PhysicsTrackerVisualizer::Ptr trackingVisualizer(new PhysicsTrackerVisualizer(&scene, alg));

  bool applyEvidence = true;
  scene.addVoidKeyCallback('a',boost::bind(toggle, &applyEvidence), "toggle apply evidence");
  scene.addVoidKeyCallback('=',boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), 0.1f), "increase opaqueness");
  scene.addVoidKeyCallback('-',boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), -0.1f), "decrease opaqueness");
  scene.addVoidKeyCallback('q',boost::bind(exit, 0), "exit");

  while (ros::ok()) {
    //Update the inputs of the featureExtractors and visibilities (if they have any inputs)
    cloudFeatures->updateInputs(filteredCloud);
    //TODO update arbitrary number of depth images)
    pending = false;
    while (ros::ok() && !pending) {
      sync.updateRobot();
//      leftGrabDetector->update(sync.m_lastMsg);
//      rightGrabDetector->update(sync.m_lastMsg);
//      lMonitor.updateGrabPose();
//      rMonitor.updateGrabPose();
        //Do iteration
      alg->updateFeatures();
      alg->expectationStep();
      alg->maximizationStep(applyEvidence);

      trackingVisualizer->update();
      scene.env->step(.03,2,.015);
      scene.draw();
      ros::spinOnce();
    }
    objPub.publish(toTrackedObjectMessage(trackedObj));
    if (RecordingConfig::record) demoRecord->frame();
  }

}
