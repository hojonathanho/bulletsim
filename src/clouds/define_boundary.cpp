#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/ros/conversions.h>
#include "clouds/pcl_typedefs.h"
#include "utils/config.h"
#include "clouds/cloud_ops.h"
#include "clouds/utils_pcl.h"
#include "clouds/utils_ros.h"
#include "utils/conversions.h"
#include "utils/utils_vector.h"
#include "utils/file.h"

using sensor_msgs::PointCloud2;
using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
	static string cloudTopic;
	static string polyTopic;
	static string filename;
	static string groundFrame;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string> ("cloudTopic", &cloudTopic, "input topic"));
    params.push_back(new Parameter<string> ("polyTopic", &polyTopic, "output topic"));
    params.push_back(new Parameter<string> ("filename", &filename, "file to which the boundary gets saved"));
    params.push_back(new Parameter<string> ("groundFrame", &groundFrame, "the polygon points are saved with respect to this frame"));
  }
};

string LocalConfig::cloudTopic = "/drop/kinect1/points";
string LocalConfig::polyTopic = "/preprocessor/kinect1/polygon";
string LocalConfig::filename = string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/boundary/polygon.bdr";
string LocalConfig::groundFrame = "/ground";

ColorCloudPtr cloud(new ColorCloud());
cv::Mat image;
vector<cv::Point2i> polygon_pixels;
geometry_msgs::Polygon polygon;
btTransform groundFromCamera;

void mouseCallback( int event, int x, int y, int flags, void* param ){
	switch( event ){
		case CV_EVENT_LBUTTONUP:
			ColorPoint pt = getCorrespondingPoint(cloud, cv::Point2i(x,y));
			if (pointIsFinite(pt)) {
				polygon_pixels.push_back(cv::Point2i(x,y));
				polygon.points.push_back(toROSPoint32(groundFromCamera * toBulletVector(pt)));
			}
	  	break;
	}
}

int main(int argc, char* argv[]) {
  Parser parser;
	parser.addGroup(GeneralConfig());
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);

  ros::init(argc, argv,"define_boundary");
  ros::NodeHandle nh;

  ros::Publisher polyPub = nh.advertise<geometry_msgs::PolygonStamped> (LocalConfig::polyTopic, 5);
	geometry_msgs::PolygonStamped polyStamped;
	polyStamped.header.frame_id = LocalConfig::groundFrame;

  tf::TransformListener listener;
	sensor_msgs::PointCloud2ConstPtr cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(LocalConfig::cloudTopic, nh);
  groundFromCamera = waitForAndGetTransform(listener, LocalConfig::groundFrame, cloud_msg->header.frame_id);

	pcl::fromROSMsg(*cloud_msg, *cloud);
  image = toCVMatImage(cloud);

  cv::namedWindow("Polygon Definition", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  cv::setMouseCallback("Polygon Definition", mouseCallback);

  cout << "Press 's' when you are done specifying the polygon points and the boundary can be saved." << endl;
  cout << "Press 'q' to quit without saving the boundary" << endl;

  char key = cv::waitKey(15);
  while (ros::ok() && key!='q' && key!='s') {
  	if (!image.empty()) {
  		for (int i=0; i<polygon_pixels.size(); i++)
  			cv::circle(image, cv::Point2i(polygon_pixels[i].x, polygon_pixels[i].y), 5, cv::Scalar(0,255,0), 1);
  		cv::imshow("Polygon Definition", image);

  		polyStamped.polygon = polygon;
  		polyStamped.header.stamp = ros::Time::now();
			polyPub.publish(polyStamped);
  	}
  	key = cv::waitKey(15);
  	ros::spinOnce();
  }
  cv::destroyWindow("Polygon Definition");
  if (key == 'q') {
  	cout << "The boundary was not saved" << endl;
  	return 0;
  }
  savePoints(LocalConfig::filename, polygon.points);
	return 0;
}
