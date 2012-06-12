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

using namespace std;
using namespace Eigen;


static std::string outputNS = "/preprocessor";
static std::string nodeName = "/preprocessor_node";

struct LocalConfig : Config {
  static std::string outputNS;
  static std::string inputTopic;
  static std::string nodeName;
  static float zClipLow;
  static float zClipHigh;
  static bool updateParams;
  static float downsample;
  static bool removeOutliers;
  static float clusterTolerance;
  static float clusterMinSize;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("inputTopic", &inputTopic, "input topic"));
    params.push_back(new Parameter<float>("zClipLow", &zClipLow, "clip points that are less than this much above table"));
    params.push_back(new Parameter<float>("zClipHigh", &zClipHigh, "clip points that are more than this much above table"));
    params.push_back(new Parameter<bool>("updateParams", &updateParams, "start a thread to periodically update the parameters thru the parameter server"));
    params.push_back(new Parameter<float>("downsample", &downsample, "downsample voxel grid size. 0 means no"));
    params.push_back(new Parameter<bool>("removeOutliers", &removeOutliers, "remove outliers"));
    params.push_back(new Parameter<float>("clusterTolerance", &clusterTolerance, "points within this distance are in the same cluster"));
    params.push_back(new Parameter<float>("clusterMinSize", &clusterMinSize, "the clusters found must have at least this number of points. 0 means no filtering"));
  }
};

string LocalConfig::inputTopic = "/camera/rgb/points";
float LocalConfig::zClipLow = .0025;
float LocalConfig::zClipHigh = 1000;
bool LocalConfig::updateParams = true;
float LocalConfig::downsample = .02;
bool LocalConfig::removeOutliers = true;
float LocalConfig::clusterTolerance = 0.03;
float LocalConfig::clusterMinSize = 40;

static int MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL;

template <typename T>
void getOrSetParam(const ros::NodeHandle& nh, std::string paramName, T& ref, T defaultVal) {
	if (!nh.getParam(paramName, ref)) {
		nh.setParam(paramName, defaultVal);
		ref = defaultVal;
		ROS_INFO_STREAM("setting " << paramName << "to default value " << defaultVal);
	}
}
void setParams(const ros::NodeHandle& nh) {
	getOrSetParam(nh, "min_hue", MIN_HUE, 160);
	getOrSetParam(nh, "max_hue", MAX_HUE, 10);
	getOrSetParam(nh, "min_sat", MIN_SAT, 150);
	getOrSetParam(nh, "max_sat", MAX_SAT, 255);
	getOrSetParam(nh, "min_val", MIN_VAL, 100);
	getOrSetParam(nh, "max_val", MAX_VAL, 255);
}

void setParamLoop() {
	ros::NodeHandle nh(nodeName);
	while (nh.ok()) {
		setParams(nh);
		sleep(1);
	}
}

class PreprocessorNode {
public:
  ros::NodeHandle& m_nh;
  ros::Publisher m_pub;
  ros::Publisher m_polyPub;
  tf::TransformBroadcaster br;
  ros::Subscriber m_sub;

  bool m_inited;

  Matrix3f m_axes;
  Vector3f m_mins, m_maxes;
	
  btTransform m_transform;
  geometry_msgs::Polygon m_poly;


  void callback(const sensor_msgs::PointCloud2& msg_in) {
    ColorCloudPtr cloud_in(new ColorCloud());
    pcl::fromROSMsg(msg_in, *cloud_in);

    if (!m_inited) {
      initTable(cloud_in);
      //ColorCloudPtr downsampled_cloud = downsampleCloud(cloud_in, LocalConfig::downsample);
      //ColorCloud* downsampled_cloud_noboost = dynamic_cast<ColorCloud*>(downsampled_cloud.get());
      //pcl::io::savePCDFile("/home/alex/rll/test/data/sample.pcd", *downsampled_cloud_noboost);
    }

    //oud_out, MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL);
    ColorCloudPtr cloud_out = checkerBoardCorners(cloud_in, 6, 7);
    //if (LocalConfig::downsample > 0) cloud_out = downsampleCloud(cloud_out, LocalConfig::downsample);
    //if (LocalConfig::removeOutliers) cloud_out = removeOutliers(cloud_out, 1, 10);
    //if (LocalConfig::clusterMinSize > 0) cloud_out = clusterFilter(cloud_out, LocalConfig::clusterTolerance, LocalConfig::clusterMinSize);

    /*
    ColorCloudPtr cloud_in_shift(new ColorCloud());
	*cloud_in_shift = *cloud_in;
	for (size_t i = 0; i < cloud_in->points.size (); ++i)
		cloud_in_shift->points[i].x = cloud_in->points[i].x + 0.07f;
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_in_shift);
	pcl::PointCloud<pcl::PointXYZRGB> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	*/


    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_out, msg_out);
    msg_out.header = msg_in.header;
    m_pub.publish(msg_out);
		
    br.sendTransform(tf::StampedTransform(m_transform, ros::Time::now(), msg_in.header.frame_id, "ground"));

    geometry_msgs::PolygonStamped polyStamped;
    polyStamped.polygon = m_poly;
    polyStamped.header.frame_id = msg_in.header.frame_id;
    polyStamped.header.stamp = ros::Time::now();
    m_polyPub.publish(polyStamped);

		
  }

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

    m_axes.col(0) = xax;
    m_axes.col(1) = yax;
    m_axes.col(2) = zax;

    MatrixXf rotCorners = corners * m_axes;

    m_mins = rotCorners.colwise().minCoeff();
    m_maxes = rotCorners.colwise().maxCoeff();
    m_mins(2) = rotCorners(0,2) + LocalConfig::zClipLow;
    m_maxes(2) = rotCorners(0,2) + LocalConfig::zClipHigh;



    m_transform.setBasis(btMatrix3x3(
		  xax(0),yax(0),zax(0),
		  xax(1),yax(1),zax(1),
		  xax(2),yax(2),zax(2)));
    m_transform.setOrigin(btVector3(corners(0,0), corners(0,1), corners(0,2)));

    m_poly.points = toROSPoints32(toBulletVectors(corners));



    m_inited = true;

  }

  PreprocessorNode(ros::NodeHandle& nh) :
    m_inited(false),
    m_nh(nh),
    m_pub(nh.advertise<sensor_msgs::PointCloud2>(outputNS+"/points",5)),
    m_polyPub(nh.advertise<geometry_msgs::PolygonStamped>(outputNS+"/polygon",5)),
    m_sub(nh.subscribe(LocalConfig::inputTopic, 1, &PreprocessorNode::callback, this))
    {
    }


  };

int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);


  ros::init(argc, argv,"preprocessor");
  ros::NodeHandle nh;

  setParams(nh);
  if (LocalConfig::updateParams) boost::thread setParamThread(setParamLoop);


  PreprocessorNode tp(nh);
  ros::spin();
}
