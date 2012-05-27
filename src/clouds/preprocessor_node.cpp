#include "utils/my_exceptions.h"
#include <ros/ros.h>
#include <pcl/point_types.h>
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

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("inputTopic", &inputTopic, "input topic"));
    params.push_back(new Parameter<float>("zClipLow", &zClipLow, "clip points that are less than this much above table"));
    params.push_back(new Parameter<float>("zClipHigh", &zClipHigh, "clip points that are more than this much above table"));
    params.push_back(new Parameter<bool>("updateParams", &updateParams, "start a thread to periodically update the parameters thru the parameter server"));
    params.push_back(new Parameter<float>("downsample", &downsample, "downsample voxel grid size. 0 means no"));
    params.push_back(new Parameter<bool>("removeOutliers", &removeOutliers, "remove outliers"));
  }
};

string LocalConfig::inputTopic = "/camera/rgb/points";
float LocalConfig::zClipLow = .0025;
float LocalConfig::zClipHigh = 1000;
bool LocalConfig::updateParams = true;
float LocalConfig::downsample = .02;
bool LocalConfig::removeOutliers = false;

static int MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL;
void updateHSVParams() {
	ros::param::param<int>(nodeName + "/min_hue", MIN_HUE, 160);
	ros::param::param<int>(nodeName + "/max_hue", MAX_HUE, 10);
	ros::param::param<int>(nodeName + "/min_sat", MIN_SAT, 100);
	ros::param::param<int>(nodeName + "/max_sat", MAX_SAT, 255);
	ros::param::param<int>(nodeName + "/min_val", MIN_VAL, 140);
	ros::param::param<int>(nodeName + "/max_val", MAX_VAL, 255);
}

void updateParamLoop() {
	ros::NodeHandle nh;
	while (nh.ok()) {
		updateHSVParams();
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
    }

    ColorCloudPtr cloud_out = orientedBoxFilter(cloud_in, m_axes, m_mins, m_maxes);
    cloud_out = hueFilter(cloud_out, MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL);
    if (LocalConfig::downsample > 0) cloud_out = downsampleCloud(cloud_out, LocalConfig::downsample);
    if (LocalConfig::removeOutliers) cloud_out = removeOutliers(cloud_out, 1.5, 10);

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_out, msg_out);
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

  updateHSVParams();
  if (LocalConfig::updateParams) boost::thread updateParamThread(updateParamLoop);


  PreprocessorNode tp(nh);
  ros::spin();
}
