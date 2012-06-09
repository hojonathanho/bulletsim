#include "utils/my_exceptions.h"
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
#include "get_table2.h"
#include <cmath>
#include "utils/config.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PolygonStamped.h>
#include "utils/conversions.h"
#include <boost/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
  static std::string inputTopic1;
  static std::string inputTopic2;
  static float zClipLow;
  static float zClipHigh;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("inputTopic1", &inputTopic1, "input topic1"));
    params.push_back(new Parameter<string>("inputTopic2", &inputTopic2, "input topic2"));
    params.push_back(new Parameter<float>("zClipLow", &zClipLow, "clip points that are less than this much above table"));
    params.push_back(new Parameter<float>("zClipHigh", &zClipHigh, "clip points that are more than this much above table"));
  }
};

string LocalConfig::inputTopic1 = "/kinect1/depth_registered/points";
string LocalConfig::inputTopic2 = "/kinect2/depth_registered/points";
float LocalConfig::zClipLow = .0025;
float LocalConfig::zClipHigh = 1000;

bool table_initialized = false;
btTransform ground_transform;
geometry_msgs::Polygon ground_poly;

boost::shared_ptr<ros::Publisher> cloudPub;
boost::shared_ptr<ros::Publisher> polyPub;
boost::shared_ptr<tf::TransformBroadcaster> broadcaster;

void initTable(ColorCloudPtr cloud) {
	MatrixXf corners = getTableCornersRansac(cloud);

	Vector3f xax = corners.row(1) - corners.row(0);
	xax.normalize();
	Vector3f yax = corners.row(3) - corners.row(0);
	yax.normalize();
	Vector3f zax = xax.cross(yax);

	float zsgn = (zax(2) > 0) ? 1 : -1;
	xax *= - zsgn;
	zax *= - zsgn; // so z axis points up

	Matrix3f axes;
	axes.col(0) = xax;
	axes.col(1) = yax;
	axes.col(2) = zax;

	MatrixXf rotCorners = corners * axes;

	Vector3f mins = rotCorners.colwise().minCoeff();
	Vector3f maxes = rotCorners.colwise().maxCoeff();
	mins(2) = rotCorners(0,2) + LocalConfig::zClipLow;
	maxes(2) = rotCorners(0,2) + LocalConfig::zClipHigh;

	ground_transform.setBasis(btMatrix3x3(xax(0),yax(0),zax(0),
										  xax(1),yax(1),zax(1),
										  xax(2),yax(2),zax(2)));
	ground_transform.setOrigin(btVector3(corners(0,0), corners(0,1), corners(0,2)));

	ground_poly.points = toROSPoints32(toBulletVectors(corners));

	table_initialized = true;
}

void callback(const sensor_msgs::PointCloud2ConstPtr& msg_in1, const sensor_msgs::PointCloud2ConstPtr& msg_in2) {
	ColorCloudPtr cloud_in1(new ColorCloud());
	ColorCloudPtr cloud_in2(new ColorCloud());
	pcl::fromROSMsg(*msg_in1, *cloud_in1);
	pcl::fromROSMsg(*msg_in2, *cloud_in2);

	if (table_initialized) {
	  initTable(cloud_in1);
	  initTable(cloud_in2);
	}

	ColorCloudPtr cloud_out(new ColorCloud(*cloud_in1));
	ColorCloud::iterator it;
	for (it = cloud_in2->begin(); it < cloud_in2->end(); it++)
		cloud_out->push_back(*it);

	sensor_msgs::PointCloud2 msg_out;
	pcl::toROSMsg(*cloud_out, msg_out);
	msg_out.header = msg_in1->header;
	cloudPub->publish(msg_out);

	broadcaster->sendTransform(tf::StampedTransform(ground_transform, ros::Time::now(), msg_in1->header.frame_id, "ground"));

	geometry_msgs::PolygonStamped polyStamped;
	polyStamped.polygon = ground_poly;
	polyStamped.header.frame_id = msg_in1->header.frame_id;
	polyStamped.header.stamp = ros::Time::now();
	polyPub->publish(polyStamped);
}

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	ros::init(argc, argv,"merger");
	ros::NodeHandle nh;

	cloudPub.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>("/merger/points",5)));
	polyPub.reset(new ros::Publisher(nh.advertise<geometry_msgs::PolygonStamped>("/merger/polygon",5)));
	broadcaster.reset(new tf::TransformBroadcaster());

	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1Sub(nh, LocalConfig::inputTopic1, 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2Sub(nh, LocalConfig::inputTopic2, 1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> cloudSync(MySyncPolicy(30), cloud1Sub, cloud2Sub);
	cloudSync.registerCallback(boost::bind(&callback,_1,_2));

	ros::spin();
}
