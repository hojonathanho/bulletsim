#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "pcl_typedefs.h"
#include "utils/config.h"

using sensor_msgs::PointCloud2;
using sensor_msgs::Image;
using namespace std;

namespace cv {
	typedef Vec<uchar, 3> Vec3b;
}

struct LocalConfig : Config {
  static string cloudTopic;
  static string depthTopic;
  static string rgbTopic;
  static string cloudFilenamePrefix;
  static string depthFilenamePrefix;
  static string rgbFilenamePrefix;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("cloudTopic", &cloudTopic, "topic of cloud to be saved"));
    params.push_back(new Parameter<string>("depthTopic", &depthTopic, "topic of depth image to be saved"));
    params.push_back(new Parameter<string>("rgbTopic", &rgbTopic, "topic of rgb image to be saved"));
    params.push_back(new Parameter<string>("cloudFilenamePrefix", &cloudFilenamePrefix, "path plus file name prefix"));
    params.push_back(new Parameter<string>("depthFilenamePrefix", &depthFilenamePrefix, "path plus file name prefix"));
    params.push_back(new Parameter<string>("rgbFilenamePrefix", &rgbFilenamePrefix, "path plus file name prefix"));
  }
};

string LocalConfig::cloudTopic = "/kinect1/depth_registered/points";
string LocalConfig::depthTopic  = "/kinect1/depth_registered/image_rect";
string LocalConfig::rgbTopic = "/kinect1/rgb/image_rect_color";
string LocalConfig::cloudFilenamePrefix = "/home/alex/rll/bulletsim/data/clouds/hand_test";
string LocalConfig::depthFilenamePrefix = "/home/alex/rll/bulletsim/data/depths/hand_test";
string LocalConfig::rgbFilenamePrefix = "/home/alex/rll/bulletsim/data/rgbs/hand_test";

int cloud_image_ind = 0;

void cloudAndImagesCallback (const sensor_msgs::PointCloud2ConstPtr& cloudMsg,
			        const sensor_msgs::ImageConstPtr& depthImageMsg,
			        const sensor_msgs::ImageConstPtr& rgbImageMsg) {

	ColorCloud cloud;
	pcl::fromROSMsg(*cloudMsg, cloud);
  cv::Mat depth_image = cv_bridge::toCvCopy(depthImageMsg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
  cv::Mat rgb_image = cv_bridge::toCvCopy(rgbImageMsg)->image;

  float d_max = 0;
  float d_min = 256.0;
  for (int i=0; i<depth_image.rows; i++) {
  	for (int j=0; j<depth_image.cols; j++) {
  		float pixel = depth_image.at<float>(i,j);
  		if (isfinite(pixel)) {
  			d_max = max(d_max, pixel);
  			d_min = min(d_min, pixel);
  		}
  	}
  }

  cout << d_min << " " << d_max << endl;
  depth_image *= 255.0/1.4;
  cv::imwrite("/home/alex/Desktop/rgb1.jpg", rgb_image);
  cv::imwrite("/home/alex/Desktop/depth1.jpg", depth_image);
  //pcl::io::savePCDFile(LocalConfig::cloudFilenamePrefix + "_" + itoa(cloud_image_ind, 4) + ".pcd", cloud);
  //cv::imwrite(LocalConfig::depthFilenamePrefix + "_" + itoa(cloud_image_ind, 4) + ".jpg", depth_image);
  //cv::imwrite(LocalConfig::rgbFilenamePrefix + "_" + itoa(cloud_image_ind, 4) + ".jpg", rgb_image);
  cloud_image_ind++;

  cout << "." << flush;
}

int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);

  ros::init(argc, argv,"save_cloud_image_seq");
  ros::NodeHandle nh;

  message_filters::Subscriber<PointCloud2> cloudSub(nh, LocalConfig::cloudTopic,1);
  message_filters::Subscriber<Image> depthImageSub(nh, LocalConfig::depthTopic, 1);
  message_filters::Subscriber<Image> rgbImageSub(nh, LocalConfig::rgbTopic, 1);
	typedef message_filters::sync_policies::ApproximateTime<PointCloud2, Image, Image> ApproxSyncPolicyAll;
	message_filters::Synchronizer<ApproxSyncPolicyAll> syncAll(ApproxSyncPolicyAll(30), cloudSub, depthImageSub, rgbImageSub);
  syncAll.registerCallback(boost::bind(&cloudAndImagesCallback,_1,_2,_3));

  ros::spin();
}
