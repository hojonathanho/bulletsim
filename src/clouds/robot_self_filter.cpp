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
static std::string nodeName = "robot_self_filter";

struct LocalConfig : Config {
  static std::string inputTopic;
  static std::string outputTopic;
  static bool colorRobotPoints;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("inputTopic", &inputTopic, "input topic"));
    params.push_back(new Parameter<string>("outputTopic", &outputTopic, "output topic"));
    params.push_back(new Parameter<float>("padding", &BulletConfig::linkPadding, "link padding"));
    params.push_back(new Parameter<bool>("colorRobotPoints", &LocalConfig::colorRobotPoints, "color robot points instead of removing"));
  }
};

string LocalConfig::inputTopic = "/drop/points";
string LocalConfig::outputTopic = "/drop/points_self_filtered";
bool LocalConfig::colorRobotPoints = false;

ros::Subscriber cloudSub, jointSub;
ros::Publisher cloudPub;
tf::TransformListener* listener;
Environment::Ptr env;
RaveRobotObject::Ptr pr2;
sensor_msgs::JointState lastJointMsg;



void cloudCallback(const sensor_msgs::PointCloud2& msg_in) {

  double tStart = GetClock();
  ColorCloudPtr cloud_in(new ColorCloud());
  pcl::fromROSMsg(msg_in, *cloud_in);

  if (msg_in.header.frame_id != "base_footprint") {
    btTransform baseFromCloud;
    if (!lookupLatestTransform(baseFromCloud, "base_footprint", msg_in.header.frame_id, *listener))  return;
    cloud_in = transformPointCloud1(cloud_in, toEigenTransform(baseFromCloud));
  }

  btTransform baseFromCam;
  if (!lookupLatestTransform(baseFromCam, "base_footprint", "camera_rgb_optical_frame", *listener)) return;
  btVector3 cameraPos = baseFromCam.getOrigin()*METERS;

  ValuesInds vi = getValuesInds(lastJointMsg.position);
  pr2->setDOFValues(vi.second, vi.first);

  ColorCloudPtr cloud_out(new ColorCloud());
  BOOST_FOREACH(ColorPoint& pt, cloud_in->points) {
    if (std::isfinite(pt.x)) {
      btVector3 target = btVector3(pt.x*METERS, pt.y*METERS, pt.z*METERS);
      btCollisionWorld::AllHitsRayResultCallback rayCallback(cameraPos, target);
      env->bullet->dynamicsWorld->rayTest(cameraPos, target, rayCallback);
      int nHits =rayCallback.m_hitFractions.size();
      if (nHits%2==0 || (rayCallback.m_hitPointWorld[nHits-1] - target).length() > BulletConfig::linkPadding*METERS) {
        cloud_out->push_back(pt);
      }
      else if (LocalConfig::colorRobotPoints) {
        pt.r=0;
        pt.g=255;
        pt.b=0;
      }
    }

  }

  if (LocalConfig::colorRobotPoints) cloud_out = cloud_in;

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*cloud_out, msg_out);
  msg_out.header.frame_id = "base_footprint";
  cloudPub.publish(msg_out);

  LOG_DEBUG_FMT("pts in: %i. pts out: %i", msg_in.width*msg_in.height, msg_out.width*msg_out.height);
  LOG_DEBUG_FMT("time: %.2f", GetClock()-tStart);

}




static bool firstJoint = true;
void jointCallback(const sensor_msgs::JointState& msg) {
  if (firstJoint) {
    setupROSRave(pr2->robot, msg);
    firstJoint=false;
  }
  lastJointMsg = msg;
}

int main(int argc, char* argv[]) {
  GeneralConfig::scale = 10;
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.addGroup(GeneralConfig());
  parser.read(argc, argv);
  ros::init(argc, argv,nodeName);

  listener = new tf::TransformListener();

  env.reset(new Environment(BulletInstance::Ptr(new BulletInstance()), OSGInstance::Ptr(new OSGInstance())));
  RaveInstance::Ptr rave(new RaveInstance());
  Load(env, rave, "robots/pr2-beta-static.zae");
  pr2 = getRobotByName(env, rave, "pr2");
  assert(pr2);
  ros::NodeHandle nh;
  cloudSub = nh.subscribe(LocalConfig::inputTopic, 1, &cloudCallback);
  jointSub = nh.subscribe("/joint_states", 1, &jointCallback);
  cloudPub = nh.advertise<sensor_msgs::PointCloud2>(LocalConfig::outputTopic, 5);

  ros::spin();

  delete listener;
}
