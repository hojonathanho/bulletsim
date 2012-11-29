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
#include "multihyp_tracker.h"
#include "initialization.h"
#include "simulation/simplescene.h"
#include "simulation/config_viewer.h"
#include "config_tracking.h"
//#include "robots/pr2.h"
//#include "robots/ros2rave.h"
//#include "robots/grabbing.h"
//#include "robots/grab_detection.h"
#include "robots/PR2Object.h"
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

string LocalConfig::cameraFrame = "/openni_rgb_optical_frame";


boost::shared_ptr<CoordinateTransformer> transformer;
boost::shared_ptr<tf::TransformListener> listener;

TrackedObject::Ptr trackedObj;
TrackedObjectFeatureExtractor::Ptr objectFeatures;
PR2Object::Ptr pr2;
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
  pr2->setJointState(*jointMsg);
  pending = true;
}

void initializeTrackedObject() {
  ros::NodeHandle nh;
  sensor_msgs::ImageConstPtr msg = ros::topic::waitForMessage<sensor_msgs::Image>("/preprocessor/kinect1/image", nh, ros::Duration(1));
  cv::Mat image, mask;
  if (!msg) {
  	ROS_WARN("couldn't get first image message");
  } else {
  	cv::Mat image_and_mask = cv_bridge::toCvCopy(msg)->image;
		extractImageAndMask(image_and_mask, image, mask);
  }
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
  GeneralConfig::scale = 100;

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

  pr2 = PR2Object::Ptr(new PR2Object(scene.rave)); // it seems to also be ok to pass in empty RaveInstance::Ptr()
	scene.env->add(pr2);
  pr2->setColor(1,1,1,.5);

//  Load(scene.env, scene.rave, EXPAND(BULLETSIM_DATA_DIR)"/xml/table.xml");
//  RaveObject::Ptr table = getObjectByName(scene.env, scene.rave, "table");
//  table->setColor(0,.8,0,.5);
//  /////HACK
//	for (int i=0; i<table->children.size(); i++) {
//	  btVector3(inertia);
//		table->children[i]->rigidBody->getCollisionShape()->calculateLocalInertia(0, inertia);
//		table->children[i]->rigidBody->setMassProps(0, inertia);
//	}
//  /////
	BoxObject::Ptr table(new BoxObject(0, btVector3(1.3,1.1,0.07)*METERS, btTransform(btQuaternion(0, 0, 0, 1), btVector3(1.4, 0, 0.7)*METERS)));
	table->setColor(0,.8,0,.5);
	scene.env->add(table);

  scene.startViewer();

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
    ros::spinOnce();
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
    if (filteredCloud) break;
    sleep(.001);
  }
  if (!ros::ok()) throw runtime_error("caught signal while waiting for first message");

  scene.manip->setTransformation(util::toOSGVector(METERS*cam_transformer.worldFromCamUnscaled.getOrigin()),
      util::toOSGVector(METERS*cam_transformer.worldFromCamUnscaled.getOrigin()+METERS*cam_transformer.worldFromCamUnscaled.getBasis().getColumn(2)),
      util::toOSGVector(-cam_transformer.worldFromCamUnscaled.getBasis().getColumn(1)));

  initializeTrackedObject();

  VisibilityInterface::Ptr visInterface(new BulletRaycastVisibility(scene.env->bullet->dynamicsWorld, &cam_transformer));

  objectFeatures.reset(new TrackedObjectFeatureExtractor(trackedObj));
  CloudFeatureExtractor::Ptr cloudFeatures(new CloudFeatureExtractor());
  //PhysicsTracker::Ptr alg(new PhysicsTracker(objectFeatures, cloudFeatures, visInterface));
  //PhysicsTracker::Ptr alg(new MultiHypTracker(objectFeatures, cloudFeatures, visInterface, grabManagers));
  //MultiHypTracker::Ptr alg_hyp(new MultiHypTracker(objectFeatures, cloudFeatures, visInterface, grabManagers));
  PhysicsTracker::Ptr alg(new StochasticPhysicsTracker(objectFeatures, cloudFeatures, visInterface, StochasticPhysicsTracker::UNIFORM, 0.0));
  PhysicsTrackerVisualizer::Ptr trackingVisualizer(new PhysicsTrackerVisualizer(&scene, alg));

  bool applyEvidence = false;
  scene.addVoidKeyCallback('a',boost::bind(toggle, &applyEvidence), "toggle apply evidence");
  scene.addVoidKeyCallback('=',boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), 0.1f), "increase opacity");
  scene.addVoidKeyCallback('-',boost::bind(&EnvironmentObject::adjustTransparency, trackedObj->getSim(), -0.1f), "decrease opacity");
  scene.addVoidKeyCallback('q',boost::bind(exit, 0), "exit");

  while (ros::ok()) {
    //Update the inputs of the featureExtractors and visibilities (if they have any inputs)
    cloudFeatures->updateInputs(filteredCloud);
    //TODO update arbitrary number of depth images)
    pending = false;
    while (ros::ok() && !pending) {
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
