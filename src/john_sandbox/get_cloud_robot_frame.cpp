#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include "clouds/utils_pcl.h"
#include "utils/conversions.h"
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include "utils/config.h"
#include <boost/foreach.hpp>
using namespace std;
struct LocalConfig : Config {
  static string cloudTopic;
  static bool useColor;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("cloudTopic", &cloudTopic, "topic with point clouds"));
    params.push_back(new Parameter<bool>("useColor", &useColor, "topic with point clouds"));
  }
};
string LocalConfig::cloudTopic = "/camera/depth_registered/points";
bool LocalConfig::useColor = true;

using namespace std;
int main(int argc, char* argv[]) {
    Parser parser;
    parser.addGroup(LocalConfig());
    parser.read(argc, argv);
	ros::init(argc, argv, "get_cloud");
	ros::NodeHandle nh;
	tf::TransformListener listener;
	string topicName = LocalConfig::cloudTopic;
	sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topicName, nh, ros::Duration(1));
	if (!msg) throw runtime_error("couldn't get a point cloud");
	if (LocalConfig::useColor) {

	}
	ColorCloudPtr cloud(new ColorCloud());
	if (LocalConfig::useColor)
      pcl::fromROSMsg(*msg, *cloud);
	else {
	  CloudPtr cloud1(new Cloud());
      pcl::fromROSMsg(*msg, *cloud1);
      BOOST_FOREACH(Point& p, cloud1->points) {
        ColorPoint cpt;
        cpt.x = p.x;
        cpt.y = p.y;
        cpt.z = p.z;
        cpt.r = 255;
        cpt.g = 255;
        cpt.b = 255;
        cloud->push_back(cpt);
      }
	}
    listener.waitForTransform("base_footprint", msg->header.frame_id, ros::Time(0), ros::Duration(1));
    tf::StampedTransform st;
    listener.lookupTransform("base_footprint", msg->header.frame_id, ros::Time(0), st);
    ColorCloudPtr cloud_tf = transformPointCloud1(cloud, toEigenTransform(st.asBt()));
    pcl::io::savePCDFileBinary("/tmp/cloud.pcd", *cloud_tf);
}
