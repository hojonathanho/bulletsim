#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "utils/config.h"
#include "clouds/utils_ros.h"
#include "utils/logging.h"
using namespace std;

struct LocalConfig: Config {
  static string kinectFrame;
  static string robotFrame;

  LocalConfig() :
    Config() {
    params.push_back(new Parameter<string> ("kinectFrame", &kinectFrame, "kinect frame"));
    params.push_back(new Parameter<string> ("robotFrame", &robotFrame, "robot frame"));
  }
};

string LocalConfig::kinectFrame = "/kinect2_link";
string LocalConfig::robotFrame = "/base_footprint";

int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);
  ros::init(argc, argv, "tf_phasespace_robot");
  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  btTransform groundFromKinect, cameraFromBase;
  ros::Rate r(10);
  while (ros::ok()) {
    for (int i = 0; i < LocalConfig::kinectFrame.size(); ++i) {
      if (lookupLatestTransform(groundFromKinect, "/ground", LocalConfig::kinectFrame, listener) &&
          lookupLatestTransform(cameraFromBase, "/camera_link", LocalConfig::robotFrame, listener)) {
	btTransform groundFromBase = groundFromKinect*cameraFromBase;
        broadcaster.sendTransform(tf::StampedTransform(groundFromBase, ros::Time::now(), "/ground", LocalConfig::robotFrame));
        LOG_DEBUG("successfully broadcasted transform");
      }
      else {
        LOG_DEBUG("transforms failed");
      }
    }
    r.sleep();
  }
}
