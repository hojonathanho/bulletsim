#include <cmath>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/filter_indices.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
#include "get_table2.h"
#include "utils_cv.h"
#include "utils/my_exceptions.h"
#include "utils/config.h"
#include "utils/conversions.h"
#include "utils_ros.h"

#include "opencv2/video/background_segm.hpp"


using namespace std;
using namespace Eigen;

static std::string nodeNS = "/preprocessor";
static std::string nodeName = "/preprocessor_node";

struct LocalConfig : Config {
  static std::string nodeNS;
  static std::string inputTopic;
  static std::string nodeName;
  static float zClipLow;
  static float zClipHigh;
  static bool updateParams;
  static float downsample;
  static bool removeOutliers;
  static float clusterTolerance;
  static float clusterMinSize;
  static float outlierRadius;
  static int outlierMinK;
  static int backgroundCount;
  static bool backgroundRead;
  static string backgroundFile;
  static int i0;
  static int i1;
  static int i2;
  static int i3;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("inputTopic", &inputTopic, "input topic"));
    params.push_back(new Parameter<float>("zClipLow", &zClipLow, "clip points that are less than this much above table"));
    params.push_back(new Parameter<float>("zClipHigh", &zClipHigh, "clip points that are more than this much above table"));
    params.push_back(new Parameter<bool>("updateParams", &updateParams, "start a thread to periodically update the parameters thru the parameter server"));
    params.push_back(new Parameter<float>("downsample", &downsample, "downsample voxel grid size. 0 means no"));
    params.push_back(new Parameter<bool>("removeOutliers", &removeOutliers, "remove outliers"));
    params.push_back(new Parameter<float>("clusterTolerance", &clusterTolerance, "points within this distance are in the same cluster"));
    params.push_back(new Parameter<float>("clusterMinSize", &clusterMinSize, "the clusters found must have at least this number of points. 0 means no filtering"));
    params.push_back(new Parameter<float>("outlierRadius", &outlierRadius, "radius search RadiusOutlierRemoval filter"));
    params.push_back(new Parameter<int>("outlierMinK", &outlierMinK, "minimum neighbors in radius search for RadiusOutlierRemoval filter"));
    params.push_back(new Parameter<int>("backgroundCount", &backgroundCount, "The first backgroundCount number of images are used for the background."));
    params.push_back(new Parameter<bool>("backgroundRead", &backgroundRead, "True if the background images should be read from file. False if the background images should be written to file."));
    params.push_back(new Parameter<string>("backgroundFile", &backgroundFile, "YAML file where the background images should be saved or read"));
    params.push_back(new Parameter<int>("i0", &i0, "miscellaneous variable 0"));
    params.push_back(new Parameter<int>("i1", &i1, "miscellaneous variable 1"));
    params.push_back(new Parameter<int>("i2", &i2, "miscellaneous variable 2"));
    params.push_back(new Parameter<int>("i3", &i3, "miscellaneous variable 3"));
  }
};

string LocalConfig::inputTopic = "/camera/rgb/points";
float LocalConfig::zClipLow = 0.0;
float LocalConfig::zClipHigh = 0.5;
bool LocalConfig::updateParams = true;
float LocalConfig::downsample = .008;
bool LocalConfig::removeOutliers = true;
float LocalConfig::clusterTolerance = 0.03;
float LocalConfig::clusterMinSize = 0;
float LocalConfig::outlierRadius = 0.016;
int LocalConfig::outlierMinK = 10;
int LocalConfig::backgroundCount = 5;
bool LocalConfig::backgroundRead = true;
string LocalConfig::backgroundFile = "/home/alex/Desktop/preprocessor.yml";
int LocalConfig::i0 = 0;
int LocalConfig::i1 = 0;
int LocalConfig::i2 = 0;
int LocalConfig::i3 = 0;

template <typename T>
void getOrSetParam(const ros::NodeHandle& nh, std::string paramName, T& ref, T defaultVal) {
	if (!nh.getParam(paramName, ref)) {
		nh.setParam(paramName, defaultVal);
		ref = defaultVal;
		ROS_INFO_STREAM("setting " << paramName << " to default value " << defaultVal);
	}
}
void setParams(const ros::NodeHandle& nh) {}

void setParamLoop(ros::NodeHandle& nh) {
	while (nh.ok()) {
		setParams(nh);
		sleep(1);
	}
}

class PreprocessorNode {
public:
  ros::NodeHandle& m_nh;
  ros::Publisher m_cloudPub;
  ros::Publisher m_polyPub;
  ros::Publisher m_foregroundPub, m_backgroundPub;
  tf::TransformBroadcaster m_broadcaster;
  tf::TransformListener m_listener;
  ros::Subscriber m_sub;

  bool m_inited;
  bool m_frame_exists;

  Matrix3f m_axes;
  Vector3f m_mins, m_maxes;
	
  btTransform m_transform;
  geometry_msgs::Polygon m_poly;

  cv::BackgroundSubtractorMOG  mog;
  vector<cv::Mat> rgb_bg, depth_bg;
  int first_count;
  cv::FileStorage fs;

  void callback(const sensor_msgs::PointCloud2& msg_in) {
    ColorCloudPtr cloud_in(new ColorCloud());
    pcl::fromROSMsg(msg_in, *cloud_in);

    if (!m_inited) {
    	initTable(cloud_in);
    }

    if (first_count>0) {
    	rgb_bg.push_back(toCVMatImage(cloud_in));
    	depth_bg.push_back(toCVMatDepthImage(cloud_in));

    	if (LocalConfig::backgroundRead) {
				fs["depth_image_" + itoa(first_count, 2)] >> depth_bg;
				fs["rgb_image_" + itoa(first_count, 2)] >> rgb_bg;
			} else {
				fs << "depth_image_" + itoa(first_count, 2) << depth_bg;
				fs << "rgb_image_" + itoa(first_count, 2) << rgb_bg;
				if (first_count == 1) fs.release();
			}

    	first_count--;
    	return;
    }

    cv::Mat rgb = toCVMatImage(cloud_in);
    cv::Mat depth = toCVMatDepthImage(cloud_in);

    ColorCloudPtr cloud_out = cloud_in;

    //RGBD image based filters
    cv::Mat foreground_RGB_mask = backgroundSubtractorColorMask(rgb, rgb_bg);
    cv::Mat foreground_depth_mask = backgroundSubtractorDepthMask(depth, depth_bg, 0.005);
    cv::Mat foreground_MOG_mask; mog(rgb, foreground_MOG_mask, 0); // zero learning rate
    cv::Mat non_skin_mask = skinMask(rgb) == 0;
		cv::Mat foreground_mask = (foreground_RGB_mask | foreground_RGB_mask | foreground_MOG_mask) & non_skin_mask;
		//cv::Mat foreground_mask = non_skin_mask;
		cloud_out = maskCloudOrganized(cloud_out, foreground_mask);

		//cloud based filters
		cloud_out = orientedBoxFilter(cloud_out, toEigenMatrix(m_transform.getBasis()), m_mins, m_maxes);
		if (LocalConfig::removeOutliers) cloud_out = removeOutliers(cloud_out, 1, 10);
		if (LocalConfig::downsample > 0) cloud_out = downsampleCloud(cloud_out, LocalConfig::downsample);
		if (LocalConfig::outlierMinK > 0) cloud_out = removeRadiusOutliers(cloud_out, LocalConfig::outlierRadius, LocalConfig::outlierMinK);
		if (LocalConfig::clusterMinSize > 0) cloud_out = clusterFilter(cloud_out, LocalConfig::clusterTolerance, LocalConfig::clusterMinSize);

		//Publish cloud
    sensor_msgs::PointCloud2 msg_out;
		pcl::toROSMsg(*cloud_out, msg_out);
		msg_out.header = msg_in.header;
		m_cloudPub.publish(msg_out);

		//Publish image based foreground/background
    cv::Mat foreground, background;
    cv::merge(vector<cv::Mat>(3, foreground_mask), foreground_mask);
    cv::multiply(rgb, foreground_mask, foreground, 1/255.0);
    cv::subtract(rgb, foreground, background);

    cv_bridge::CvImage foreground_msg;
    foreground_msg.header   = msg_in.header;
    foreground_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    foreground_msg.image    = foreground;
    m_foregroundPub.publish(foreground_msg.toImageMsg());

    cv_bridge::CvImage background_msg;
    background_msg.header   = msg_in.header;
    background_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    background_msg.image    = background;
    m_backgroundPub.publish(background_msg.toImageMsg());


		broadcastKinectTransform(m_transform.inverse(), msg_in.header.frame_id, "ground", m_broadcaster, m_listener);

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

    //if chess_board frame id exists, then z axis is already pointing up
    float zsgn = (m_listener.frameExists("chess_board") || (zax(2) < 0)) ? 1 : -1;
    xax *= zsgn;
    zax *= zsgn; // so z axis points up

    m_axes.col(0) = xax;
    m_axes.col(1) = yax;
    m_axes.col(2) = zax;

    MatrixXf rotCorners = corners * m_axes;

    m_mins = rotCorners.colwise().minCoeff();
    m_maxes = rotCorners.colwise().maxCoeff();
    m_mins(0) += 0.03;
    m_mins(1) += 0.03;
    m_maxes(0) -= 0.03;
    m_maxes(1) -= 0.03;
    m_mins(2) = rotCorners(0,2) + LocalConfig::zClipLow;
    m_maxes(2) = rotCorners(0,2) + LocalConfig::zClipHigh;

    m_transform.setBasis(toBulletMatrix(m_axes));
    m_transform.setOrigin(btVector3(corners(0,0), corners(0,1), corners(0,2)));

    m_poly.points = toROSPoints32(toBulletVectors(corners));

    m_inited = true;
  }

  PreprocessorNode(ros::NodeHandle& nh) :
    m_inited(false),
    m_nh(nh),
    m_cloudPub(nh.advertise<sensor_msgs::PointCloud2>(nodeNS+"/points",5)),
    m_polyPub(nh.advertise<geometry_msgs::PolygonStamped>(nodeNS+"/polygon",5)),
    m_foregroundPub(nh.advertise<sensor_msgs::Image>(nodeNS+"/foreground",5)),
    m_backgroundPub(nh.advertise<sensor_msgs::Image>(nodeNS+"/background",5)),
    m_sub(nh.subscribe(LocalConfig::inputTopic, 1, &PreprocessorNode::callback, this)),
		m_mins(-10,-10,-10),
		m_maxes(10,10,10),
		m_transform(toBulletTransform(Affine3f::Identity())),
		first_count(LocalConfig::backgroundCount)
    {
			if (LocalConfig::backgroundRead) {
					fs = cv::FileStorage(LocalConfig::backgroundFile, cv::FileStorage::READ);
				} else {
					fs = cv::FileStorage(LocalConfig::backgroundFile, cv::FileStorage::WRITE);
			}
    }
};

int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);

  ros::init(argc, argv,"preprocessor");
  ros::NodeHandle nh(nodeNS);

  setParams(nh);
  if (LocalConfig::updateParams) boost::thread setParamThread(setParamLoop, nh);

  PreprocessorNode tp(nh);
  ros::spin();
}
