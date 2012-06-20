#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include "clouds/utils_pcl.h"
#include "utils/config.h"

using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
  static std::string inputTopic;
  static std::string outputFilename;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("inputTopic", &inputTopic, "input topic"));
    params.push_back(new Parameter<string>("outputFilename", &outputFilename, "output file name"));
  }
};

string LocalConfig::inputTopic = "/merger/points";
string LocalConfig::outputFilename = "/home/alex/Desktop/cloud.pcd";

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	ros::init(argc, argv,"save_cloud");
	ros::NodeHandle nh;
	sensor_msgs::PointCloud2ConstPtr msg_in = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(LocalConfig::inputTopic, nh);

	ColorCloudPtr cloud_in(new ColorCloud());
	pcl::fromROSMsg(*msg_in, *cloud_in);
	pcl::io::savePCDFile(LocalConfig::outputFilename, *cloud_in);

	return 0;
}
