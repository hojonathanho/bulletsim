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
#include "simulation/config_viewer.h"
#include "config_tracking.h"
#include "robots/pr2.h"
#include "robots/ros2rave.h"
#include "robots/grabbing.h"
#include "robots/grab_detection.h"
#include "simulation/bullet_io.h"
#include "clouds/ros_robot.h"
#include <std_srvs/Empty.h>
#include "demo_recorder.h"
#include "simulation/softbodies.h"
#include "clouds/utils_cv.h"

struct LocalConfig: Config {
  static std::string cameraFrame;

  LocalConfig() :
    Config() {
    params.push_back(new Parameter<string> ("cameraFrame", &cameraFrame, "camera frame"));
  }
};

string LocalConfig::cameraFrame = "/camera_rgb_optical_frame";


boost::shared_ptr<CoordinateTransformer> transformer;
boost::shared_ptr<tf::TransformListener> listener;
boost::shared_ptr<RobotSync> robotSync;

TrackedObject::Ptr trackedObj;
TrackedObjectFeatureExtractor::Ptr objectFeatures;
Environment::Ptr env;

bool pending = false;
ColorCloudPtr filteredCloud;
ros::Time lastTime;
void callback (const sensor_msgs::PointCloud2ConstPtr& cloudMsg, const sensor_msgs::JointStateConstPtr& jointMsg) {
  lastTime = cloudMsg->header.stamp;
  btTransform wfc;
  if (!lookupLatestTransform(wfc, "base_footprint", cloudMsg->header.frame_id, *listener)) return;
  transformer->reset(wfc);
//  filteredCloud = fromROSMsg1(*cloudMsg);
  filteredCloud.reset(new ColorCloud());
  pcl::fromROSMsg(*cloudMsg, *filteredCloud);
  pcl::transformPointCloud(*filteredCloud, *filteredCloud, transformer->worldFromCamEigen);
  robotSync->jointCB(*jointMsg);
  robotSync->updateRobot();
  pending = true;
}

void initializeTrackedObject() {
  ros::NodeHandle nh;
  sensor_msgs::ImageConstPtr msg = ros::topic::waitForMessage<sensor_msgs::Image>("/preprocessor/kinect1/image", nh, ros::Duration(1));
  if (!msg) throw std::runtime_error("initializeTrackedObject: couldn't get first image message");
  cv::Mat image_and_mask = cv_bridge::toCvCopy(msg)->image;
  cv::Mat image, mask;
  extractImageAndMask(image_and_mask, image, mask);
  trackedObj = callInitServiceAndCreateObject(filteredCloud, image, mask, transformer.get());
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
  parser.addGroup(ViewerConfig());
  parser.addGroup(LocalConfig());
  GeneralConfig::scale = 10;
  parser.read(argc, argv);

  ros::init(argc, argv,"tracker_node");
  ros::NodeHandle nh;

  transformer.reset(new CoordinateTransformer());
  listener.reset(new tf::TransformListener());

  Scene scene;
  env = scene.env;
  util::setGlobalEnv(scene.env);
  PR2Manager pr2m(scene);
  Load(scene.env, scene.rave, EXPAND(BULLETSIM_DATA_DIR)"/xml/table.xml");
  RaveObject::Ptr table = getObjectByName(scene.env, scene.rave, "table");
  table->setColor(0,.8,0,.5);
  pr2m.pr2->setColor(1,1,1,.5);
  scene.startViewer();

  robotSync.reset(new RobotSync(nh, pr2m.pr2, false));

  syncAndRegCloudJoint(TrackingConfig::filteredCloudTopic, nh, &callback);

  ros::Publisher objPub = nh.advertise<bulletsim_msgs::TrackedObject>(trackedObjectTopic,10);
  ros::ServiceServer resetSrv = nh.advertiseService("/tracker/reinitialize",   reinitialize);



  boost::shared_ptr<ScreenThreadRecorder> screen_recorder;
  boost::shared_ptr<ImageTopicRecorder> image_topic_recorder;
  if (RecordingConfig::record == RECORD_RENDER_ONLY) {
        screen_recorder.reset(new ScreenThreadRecorder(scene.viewer, RecordingConfig::dir + "/" +  RecordingConfig::video_file + "_tracked.avi"));
  } else if (RecordingConfig::record == RECORD_RENDER_AND_TOPIC) {
        screen_recorder.reset(new ScreenThreadRecorder(scene.viewer, RecordingConfig::dir + "/" +  RecordingConfig::video_file + "_tracked.avi"));
        image_topic_recorder.reset(new ImageTopicRecorder(nh, "/preprocessor" + TrackingConfig::cameraTopics[0] + "/image", RecordingConfig::dir + "/" +  RecordingConfig::video_file + "_topic.avi"));
  }

  CoordinateTransformer cam_transformer;
  btTransform baseFromCam;
  while (ros::ok()) {
    if (lookupLatestTransform(baseFromCam, "base_footprint", LocalConfig::cameraFrame, *listener)) {
      cam_transformer.reset(baseFromCam);
      break;
    }
    else {
      LOG_DEBUG("waiting for tf to work");
      sleep(.04);
    }
  }
  while (ros::ok()) {
    ros::spinOnce();
    if (filteredCloud && robotSync->m_lastMsg.position.size()>0) break;
    sleep(.001);
  }
  if (!ros::ok()) throw runtime_error("caught signal while waiting for first message");

  scene.manip->setTransformation(util::toOSGVector(METERS*cam_transformer.worldFromCamUnscaled.getOrigin()),
      util::toOSGVector(METERS*cam_transformer.worldFromCamUnscaled.getOrigin()+METERS*cam_transformer.worldFromCamUnscaled.getBasis().getColumn(2)),
      util::toOSGVector(-cam_transformer.worldFromCamUnscaled.getBasis().getColumn(1)));

  initializeTrackedObject();
  GrabManager lgm, rgm;
  BulletSoftObject::Ptr maybeBSO = boost::dynamic_pointer_cast<BulletSoftObject>(trackedObj->m_sim);
  if (maybeBSO){
    lgm = GrabManager(scene.env, pr2m.pr2, pr2m.pr2Left, GrabDetector::LEFT, robotSync.get(), maybeBSO);
    rgm = GrabManager(scene.env, pr2m.pr2, pr2m.pr2Right, GrabDetector::RIGHT, robotSync.get(), maybeBSO);
  }
  else {
    lgm = GrabManager(scene.env, pr2m.pr2Left, GrabDetector::LEFT, robotSync.get());
    rgm = GrabManager(scene.env, pr2m.pr2Right, GrabDetector::RIGHT, robotSync.get());
  }


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
      lgm.update();
      rgm.update();

      alg->updateFeatures();
      alg->expectationStep();
      alg->maximizationStep(applyEvidence);

      trackingVisualizer->update();
      scene.env->step(.03,2,.015);
      scene.draw();
      ros::spinOnce();
    }
    bulletsim_msgs::TrackedObject objOut = toTrackedObjectMessage(trackedObj);
    objOut.header.stamp = lastTime;
    objPub.publish(objOut);
  }

}
