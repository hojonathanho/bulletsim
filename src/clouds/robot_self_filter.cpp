#include "robots/pr2.h"
#include "clouds/cloud_ops.h"
#include "clouds/utils_pcl.h"
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include "utils/logging.h"
#include "simulation/environment.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl/ros/conversions.h>
#include "utils/conversions.h"
#include "robots/ros2rave.h"
#include "utils/clock.h"
#include "utils_ros.h"
#include "ros_robot.h"
#include "tracking/plotting_tracking.h"
#include "tracking/utils_tracking.h"

static std::string nodeName = "robot_self_filter";

struct LocalConfig: Config {
  static std::string inputTopic;
  static std::string outputTopic;
  static std::string cameraFrame;
  static bool colorRobotPoints;

  LocalConfig() :
    Config() {
    params.push_back(new Parameter<string> ("inputTopic", &inputTopic, "input topic"));
    params.push_back(new Parameter<string> ("outputTopic", &outputTopic, "output topic"));
    params.push_back(new Parameter<string> ("cameraFrame", &cameraFrame, "camera frame"));
    params.push_back(new Parameter<float> ("padding", &BulletConfig::linkPadding, "link padding"));
    params.push_back(new Parameter<bool> ("colorRobotPoints", &LocalConfig::colorRobotPoints, "color robot points instead of removing"));
  }
};

string LocalConfig::inputTopic = "/drop/points";
string LocalConfig::outputTopic = "/drop/points_self_filtered";
string LocalConfig::cameraFrame = "/camera_rgb_optical_frame";
bool LocalConfig::colorRobotPoints = false;

boost::shared_ptr<tf::TransformListener> listener;
boost::shared_ptr<RobotSync> robotSync;
RaveRobotObject::Ptr pr2;
sensor_msgs::JointState lastJointMsg;
ros::Publisher cloudPub;
ColorCloudPtr cloud_in, cloud_out;
Environment::Ptr env;

//#define PLOT_RAYCAST

void callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg, const sensor_msgs::JointStateConstPtr& jointMsg) {

  robotSync->jointCB(*jointMsg);
  robotSync->updateRobot();

  double tStart = GetClock();
  cloud_in.reset(new ColorCloud());
  pcl::fromROSMsg(*cloudMsg, *cloud_in);

  if (cloudMsg->header.frame_id != "base_footprint") {
    btTransform baseFromCloud;
    if (!lookupLatestTransform(baseFromCloud, "base_footprint", cloudMsg->header.frame_id, *listener)) return;
    cloud_in = transformPointCloud1(cloud_in, toEigenTransform(baseFromCloud));
  }

  btTransform baseFromCam;
  if (!lookupLatestTransform(baseFromCam, "base_footprint", LocalConfig::cameraFrame, *listener)) return;
  btVector3 cameraPos = baseFromCam.getOrigin() * METERS;

#ifdef PLOT_RAYCAST
  static PlotLines::Ptr rayTestLines;
  static PlotPoints::Ptr rayHitPoints;
  if (!rayTestLines) {
    rayTestLines.reset(new PlotLines(4));
    rayHitPoints.reset(new PlotPoints(20));
    rayTestLines->setDefaultColor(0, 0, 1, .5);
    rayHitPoints->setDefaultColor(1, 0, 0, .5);
    env->add(rayTestLines);
    env->add(rayHitPoints);
    //      rayTestLines->setDefaultColor(0,0,1,.5);
    //      rayHitPoints->setDefaultColor(1,0,0,.5);
    printf("adding ray test lines to env\n");
  }
  vector<btVector3> linePoints;
  vector<btVector3> hitPoints;
#endif

  cloud_out.reset(new ColorCloud());
  BOOST_FOREACH(ColorPoint& pt, cloud_in->points) {
    if (std::isfinite(pt.x)) {
      btVector3 target = btVector3(pt.x * METERS, pt.y * METERS, pt.z * METERS);
      btCollisionWorld::AllHitsRayResultCallback rayCallback(cameraPos, target);
      //      btCollisionWorld::ClosestRayResultCallback rayCallback(cameraPos, target);
      env->bullet->dynamicsWorld->rayTest(cameraPos, target, rayCallback);
      int nHits = rayCallback.m_hitFractions.size();
      if (nHits==0 || (rayCallback.m_hitPointWorld[nHits-1] - target).length() > 3*BulletConfig::linkPadding*METERS) {
        cloud_out->push_back(pt);
      } else if (LocalConfig::colorRobotPoints) {
        pt.r = 0;
        pt.g = 255;
        pt.b = 0;
      }

#ifdef PLOT_RAYCAST
      for (int i = 0; i < nHits; ++i)
      hitPoints.push_back(rayCallback.m_hitPointWorld[i]);
      linePoints.push_back(cameraPos);
      linePoints.push_back(target);
#endif

    }

  }

#ifdef PLOT_RAYCAST
  rayTestLines->setPoints(linePoints);
  rayHitPoints->setPoints(hitPoints);
#endif

  if (LocalConfig::colorRobotPoints) cloud_out = cloud_in;

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*cloud_out, msg_out);
  msg_out.header.frame_id = "base_footprint";
  msg_out.header.stamp = cloudMsg->header.stamp;
  cloudPub.publish(msg_out);

  LOG_DEBUG_FMT("pts in: %i. pts out: %i", cloudMsg->width*cloudMsg->height, msg_out.width*msg_out.height);
  LOG_DEBUG_FMT("time: %.2f", GetClock()-tStart);

}

int main(int argc, char* argv[]) {
  GeneralConfig::scale = 10;
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.addGroup(GeneralConfig());
  parser.read(argc, argv);
  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh;
#define PLOT_RAYCAST

#ifdef PLOT_RAYCAST
  Scene scene;
  env = scene.env;
  Load(scene.env, scene.rave, "robots/pr2-beta-static.zae");
  pr2 = getRobotByName(env, scene.rave, "pr2");
#else
  env.reset(new Environment(BulletInstance::Ptr(new BulletInstance()), OSGInstance::Ptr(new OSGInstance())));
  RaveInstance::Ptr rave(new RaveInstance());
  Load(env, rave, "robots/pr2-beta-static.zae");
  pr2 = getRobotByName(env, rave, "pr2");
#endif


  assert(pr2);
  pr2->setColor(1, 1, 1, .5);


#ifdef PLOT_RAYCAST
  PointCloudPlot::Ptr plotpts(new PointCloudPlot(6));
  scene.env->add(plotpts);
  scene.startViewer();
#endif

  robotSync.reset(new RobotSync(nh, pr2, false));
  listener.reset(new tf::TransformListener());
  cloudPub = nh.advertise<sensor_msgs::PointCloud2> (LocalConfig::outputTopic, 5);
  syncAndRegCloudJoint(LocalConfig::inputTopic, nh, &callback);

#ifdef PLOT_RAYCAST
  while (ros::ok()) {
    ros::spinOnce();
    if (cloud_in) plotpts->setPoints1(scaleCloud(cloud_out, METERS), 1);
    scene.step(.05);
    sleep(.05);
  }
#else
  ros::spin();
#endif

}
