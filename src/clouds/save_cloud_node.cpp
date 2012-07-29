#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include "clouds/utils_pcl.h"
#include "utils/config.h"
#include "utils/conversions.h"

using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
  static std::string inputTopic1;
  static std::string inputTopic2;
  static std::string outputFilename;
  static std::string frameId;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("inputTopic1", &inputTopic1, "The cloud of this topic always gets saved."));
    params.push_back(new Parameter<string>("inputTopic2", &inputTopic2, "The cloud of this topic gets saved only if this option is not '/'."));
    params.push_back(new Parameter<string>("outputFilename", &outputFilename, "Output file name."));
    params.push_back(new Parameter<string>("frameId", &frameId, "The point clouds(s) are transformed into this coordinate frame if this option is not '/'."));
  }
};

string LocalConfig::inputTopic1 = "/kinect1/depth_registered/points";
string LocalConfig::inputTopic2 = "/kinect2/depth_registered/points";
string LocalConfig::outputFilename = "/home/alex/Desktop/cloud.pcd";
string LocalConfig::frameId = "ground";

ColorCloudPtr getAndTransformPointCloud(sensor_msgs::PointCloud2ConstPtr msg, string frame="/") {
	ColorCloudPtr cloud(new ColorCloud());
	pcl::fromROSMsg(*msg, *cloud);
	if (frame != "/") {
		tf::StampedTransform transform;
		tf::TransformListener listener;

		while(1) {
			try {
				listener.lookupTransform (frame, msg->header.frame_id, ros::Time(0), transform);
			} catch (...) {
				continue;
			}
			break;
		}

		pcl::transformPointCloud(*cloud.get(), *cloud.get(), toEigenTransform(transform.asBt()));
	}
	return cloud;
}

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	ros::init(argc, argv,"save_cloud");
	ros::NodeHandle nh;

	sensor_msgs::PointCloud2ConstPtr cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(LocalConfig::inputTopic1, nh);
	ColorCloudPtr cloud = getAndTransformPointCloud(cloud_msg, LocalConfig::frameId);

	if (LocalConfig::inputTopic2 != "/") {
		sensor_msgs::PointCloud2ConstPtr cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(LocalConfig::inputTopic2, nh);
		*cloud += *getAndTransformPointCloud(cloud_msg, LocalConfig::frameId);
	}

	pcl::io::savePCDFile(LocalConfig::outputFilename, *cloud);

	return 0;
}
