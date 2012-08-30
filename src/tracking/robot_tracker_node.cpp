#include <sensor_msgs/JointState.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <bulletsim_msgs/TrackedObject.h>
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

CoordinateTransformer* transformer;
tf::TransformListener* listener;

bool pending = false;
ColorCloudPtr filteredCloud(new ColorCloud());
void cloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
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

  filteredCloud = fromROSMsg1(*cloudMsg);
  pcl::transformPointCloud(*filteredCloud, *filteredCloud, transformer->worldFromCamEigen);
  pending = true;
}

sensor_msgs::JointState lastJointMsg;
bool jointPending = false;
void jointCallback(const sensor_msgs::JointState& msg) {
  lastJointMsg = msg;
  jointPending = true;
//  if (leftGrabDetector != NULL) leftGrabDetector->update(msg);
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
  listener = new tf::TransformListener();

  ros::Subscriber cloudSub = nh.subscribe(TrackingConfig::filteredCloudTopic, 1, &cloudCallback);
  ros::Subscriber jointSub = nh.subscribe("/joint_states", 1, jointCallback);
  ros::Publisher objPub = nh.advertise<bulletsim_msgs::TrackedObject>(trackedObjectTopic,10);

  while (!pending || lastJointMsg.position.size() == 0 ) {
    ros::spinOnce();
    sleep(.001);
    if (!ros::ok()) throw runtime_error("caught signal while waiting for first message");
  }

  CoordinateTransformer cam_transformer;
  tf::StampedTransform st;
  while (true) {
    try {
      listener->lookupTransform("base_footprint", "camera_rgb_optical_frame", ros::Time(0), st);
      cam_transformer.reset(st.asBt());
      break;
    }
    catch (tf::TransformException e) {
      printf("tf error\n");
      sleep(.1);
    }
  }


  // set up scene
  Scene scene;
  PR2Manager pr2m(scene);
  setGlobalEnv(scene.env);
  setupROSRave(pr2m.pr2->robot, lastJointMsg);
  Load(scene.env, scene.rave, EXPAND(BULLETSIM_DATA_DIR)"/xml/table.xml");
  RaveObject::Ptr table = getObjectByName(scene.env, scene.rave, "table");
  table->setColor(0,.8,0,.5);
  pr2m.pr2->setColor(1,1,1,.5);
  cout << "table rb: " << table->children[0]->rigidBody.get() << endl;;

  MonitorForGrabbing lMonitor(pr2m.pr2Left, scene.env->bullet->dynamicsWorld);
  MonitorForGrabbing rMonitor(pr2m.pr2Right, scene.env->bullet->dynamicsWorld);
  GrabDetector* leftGrabDetector = new GrabDetector(GrabDetector::LEFT,
          boost::bind(&MonitorForGrabbing::grab, &lMonitor),
          boost::bind(&MonitorForGrabbing::release, &lMonitor));
  GrabDetector* rightGrabDetector = new GrabDetector(GrabDetector::RIGHT,
          boost::bind(&MonitorForGrabbing::grab, &rMonitor),
          boost::bind(&MonitorForGrabbing::release, &rMonitor));

  scene.startViewer();

// xxx: I'm passing bad arguments to callInitService
  TrackedObject::Ptr trackedObj = callInitServiceAndCreateObject(scaleCloud(filteredCloud,1/METERS), cv::Mat(), 0);
  if (!trackedObj) throw runtime_error("initialization of object failed.");
  trackedObj->init();
  scene.env->add(trackedObj->m_sim);

  CapsuleRope* maybeRope = dynamic_cast<CapsuleRope*>(trackedObj->m_sim.get());
  if (maybeRope) {
    printf("setting children\n");
    lMonitor.setBodies(maybeRope->getChildren());
    rMonitor.setBodies(maybeRope->getChildren());
  }

  VisibilityInterface::Ptr visInterface(new BulletRaycastVisibility(scene.env->bullet->dynamicsWorld, &cam_transformer));

  TrackedObjectFeatureExtractor::Ptr objectFeatures(new TrackedObjectFeatureExtractor(trackedObj));
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
      if (jointPending) {
        ValuesInds vi = getValuesInds(lastJointMsg.position);
        pr2m.pr2->setDOFValues(vi.second, vi.first);
        leftGrabDetector->update(lastJointMsg);
        rightGrabDetector->update(lastJointMsg);
        lMonitor.updateGrabPose();
        rMonitor.updateGrabPose();
        jointPending = false;
      }
        //Do iteration
      alg->updateFeatures();
      alg->expectationStep();
      alg->maximizationStep(applyEvidence);

      trackingVisualizer->update();
      scene.env->step(.03,2,.015);
      scene.draw();
      ros::spinOnce();
    }
  }

}
