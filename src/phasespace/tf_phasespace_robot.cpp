#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "utils/config.h"
#include "clouds/utils_ros.h"
#include "utils/logging.h"
using namespace std;

struct LocalConfig: Config {
  static vector<string> kinectFrames;
  static string robotFrame;

  LocalConfig() :
    Config() {
    params.push_back(new ParameterVec<string> ("kinectFrames", &kinectFrames, "kinect frames"));
    params.push_back(new Parameter<string> ("robotFrame", &robotFrame, "robot frame"));
  }
};

static const string kinectFrames_a[] = { "/kinect1_link", "/kinect2_link" };
vector<string> LocalConfig::kinectFrames = std::vector<string>(kinectFrames_a, kinectFrames_a+sizeof(kinectFrames_a)/sizeof(string));
string LocalConfig::robotFrame = "/odom_combined";

int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);
  ros::init(argc, argv, "tf_phasespace_robot");
  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  btTransform groundFromKinect, baseFromCamera;
  while (ros::ok()) {
    for (int i = 0; i < LocalConfig::kinectFrames.size(); ++i) {
      if (lookupLatestTransform(groundFromKinect, "/ground", LocalConfig::kinectFrames[i], listener) &&
          lookupLatestTransform(baseFromCamera, LocalConfig::robotFrame, "/camera_link", listener)) {
        broadcaster.sendTransform(tf::StampedTransform(groundFromKinect*baseFromCamera.inverse(), ros::Time::now(), "/ground", LocalConfig::robotFrame));
        LOG_DEBUG("successfully broadcasted transform");
      }
      else {
        LOG_DEBUG("transforms failed");
      }
    }
    sleep(.1);
  }
}
