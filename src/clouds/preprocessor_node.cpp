#include "utils/my_exceptions.h"
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include <pcl/surface/convex_hull.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
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
  static float skinRadius;
  static int skinColorDist;
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
    params.push_back(new Parameter<float>("skinRadius", &skinRadius, "skin color segmentation: nearest neighbor search radius to find other points that are potentially part of a skin"));
    params.push_back(new Parameter<int>("skinColorDist", &skinColorDist, "skin color segmentation: RGB [0:255] squared distance to accept or reject the points from the nearest neighbor search"));
    params.push_back(new Parameter<int>("i0", &i0, "i0"));
    params.push_back(new Parameter<int>("i1", &i1, "i1"));
    params.push_back(new Parameter<int>("i2", &i2, "i2"));
    params.push_back(new Parameter<int>("i3", &i3, "i3"));
  }
};

string LocalConfig::inputTopic = "/camera/rgb/points";
float LocalConfig::zClipLow = 0.0;
float LocalConfig::zClipHigh = 0.5;
bool LocalConfig::updateParams = true;
float LocalConfig::downsample = .005;
bool LocalConfig::removeOutliers = true;
float LocalConfig::clusterTolerance = 0.03;
float LocalConfig::clusterMinSize = 0;
float LocalConfig::skinRadius = 0.08;
int LocalConfig::skinColorDist = 8000;
int LocalConfig::i0 = 0;
int LocalConfig::i1 = 0;
int LocalConfig::i2 = 0;
int LocalConfig::i3 = 0;

static int MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL;
static int MIN_L, MAX_L, MIN_A, MAX_A, MIN_B, MAX_B;

template <typename T>
void getOrSetParam(const ros::NodeHandle& nh, std::string paramName, T& ref, T defaultVal) {
	if (!nh.getParam(paramName, ref)) {
		nh.setParam(paramName, defaultVal);
		ref = defaultVal;
		ROS_INFO_STREAM("setting " << paramName << "to default value " << defaultVal);
	}
}
void setParams(const ros::NodeHandle& nh) {
	getOrSetParam(nh, "min_hue", MIN_HUE, 100);
	getOrSetParam(nh, "max_hue", MAX_HUE, 40);
	getOrSetParam(nh, "min_sat", MIN_SAT, 100);
	getOrSetParam(nh, "max_sat", MAX_SAT, 255);
	getOrSetParam(nh, "min_val", MIN_VAL, 0);
	getOrSetParam(nh, "max_val", MAX_VAL, 255);

	getOrSetParam(nh, "min_l", MIN_L, 30);
	getOrSetParam(nh, "max_l", MAX_L, 255);
	getOrSetParam(nh, "min_a", MIN_A, 115);
	getOrSetParam(nh, "max_a", MAX_A, 255);
	getOrSetParam(nh, "min_b", MIN_B, 0);
	getOrSetParam(nh, "max_b", MAX_B, 255);
}

void setParamLoop() {
	ros::NodeHandle nh(nodeName);
	while (nh.ok()) {
		setParams(nh);
		sleep(1);
	}
}

void printImage(cv::Mat image) {
	for (int j=0; j<image.cols; j++)
		printf("j=%d %d %d %d\n", j, image.at<cv::Vec3b>(0,j)[0], image.at<cv::Vec3b>(0,j)[1], image.at<cv::Vec3b>(0,j)[2]);
}

double clipColor(float c) {
	if (c<0) c = 0;
	if(c>255) c = 255;
	return c;
}

class PreprocessorNode {
public:
  ros::NodeHandle& m_nh;
  ros::Publisher m_pubCloud;
  ros::Publisher m_pubCloudBorder;
  ros::Publisher m_pubCloudComp;
  ros::Publisher m_polyPub;
  tf::TransformBroadcaster br;
  ros::Subscriber m_sub;
  ros::Subscriber m_subImg;
  ros::Publisher m_pubImg1;
  ros::Publisher m_pubImg2;
  ros::Publisher m_pubImg3;

  bool m_inited;

  Matrix3f m_axes;
  Vector3f m_mins, m_maxes;
	
  btTransform m_transform;
  geometry_msgs::Polygon m_poly;


  void callback(const sensor_msgs::PointCloud2& msg_in) {
    ColorCloudPtr cloud_in(new ColorCloud());
    pcl::fromROSMsg(msg_in, *cloud_in);

    if (!m_inited) {
    	ColorCloudPtr cloud_green = colorSpaceFilter(cloud_in, 0, 255, 100, 255, 0, 255, CV_BGR2Lab, true);
    	cloud_green = clusterFilter(cloud_green, 0.01, 100);
    	if (cloud_green->size() < 50) {
    		ROS_WARN("The green table cannot be seen. The table couldn't be initialized.");
    	} else {
    		initTable(cloud_green);
    	}
    }

    //ColorCloudPtr cloud_out(new ColorCloud());
    ColorCloudPtr cloud_out = cloud_in;

//    if (LocalConfig::downsample > 0) cloud_out = downsampleCloud(cloud_out, LocalConfig::downsample);
		cloud_out = orientedBoxFilter(cloud_out, toEigenMatrix(m_transform.getBasis()), m_mins, m_maxes);

		//Filter out green background and put yellow back
		ColorCloudPtr cloud_neg_green = colorSpaceFilter(cloud_out, MIN_L, MAX_L, MIN_A, MAX_A, MIN_B, MAX_B, CV_BGR2Lab);
		ColorCloudPtr cloud_yellow = colorSpaceFilter(cloud_out, 0, 255, 0, 255, 190, 255, CV_BGR2Lab);
		*cloud_out = *cloud_neg_green + *cloud_yellow;
		if (LocalConfig::downsample > 0) cloud_out = downsampleCloud(cloud_out, LocalConfig::downsample);

		//Filter out hands and skin-color objects
		//input cloud for skinFilter has to be dense
//		ColorCloudPtr cloud_skin = skinFilter(cloud_in);
//		cloud_skin = downsampleCloud(cloud_skin, 0.02);
//		cloud_out = filterNeighbors(cloud_out, cloud_skin, LocalConfig::skinRadius, LocalConfig::skinColorDist, true);

		if (LocalConfig::removeOutliers) cloud_out = removeOutliers(cloud_out, 1, 10);
		if (LocalConfig::clusterMinSize > 0) cloud_out = clusterFilter(cloud_out, LocalConfig::clusterTolerance, LocalConfig::clusterMinSize);

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_out, msg_out);
    msg_out.header = msg_in.header;
    m_pubCloud.publish(msg_out);

		cloud_yellow = colorSpaceFilter(cloud_out, 0, 255, 0, 255, 190, 255, CV_BGR2Lab);
		sensor_msgs::PointCloud2 msg_out_border;
		pcl::toROSMsg(*cloud_yellow, msg_out_border);
		msg_out_border.header = msg_in.header;
		m_pubCloudBorder.publish(msg_out_border);

//    sensor_msgs::PointCloud2 msg_out_comp;
//		pcl::toROSMsg(*cloud_skin, msg_out_comp);
//		msg_out_comp.header = msg_in.header;
//		m_pubCloudComp.publish(msg_out_comp);

    br.sendTransform(tf::StampedTransform(m_transform, ros::Time::now(), msg_in.header.frame_id, "ground"));

//    MatrixXu bgr = toBGR(cloud_in);
//    cv::Mat image(cloud_in->height,cloud_in->width, CV_8UC3, bgr.data());
//    cv::imwrite("/home/alex/Desktop/yellow.jpg", image);

//		MatrixXu bgr = toBGR(cloud_in);
//	  cv::Mat image(cloud_in->height,cloud_in->width, CV_8UC3, bgr.data());
//	  //cv::Mat image = cv::imread("/home/alex/Desktop/data.jpg");
//		cv::Mat imageYCrCb(image.rows, image.cols, CV_8UC3);
//		cv::cvtColor(image, imageYCrCb, CV_BGR2YCrCb);
//		cv::Mat imageLab(image.rows, image.cols, CV_8UC3);
//		cv::cvtColor(image, imageLab, CV_BGR2Lab);
//		cv::Mat imageHSV(image.rows, image.cols, CV_8UC3);
//		cv::cvtColor(image, imageHSV, CV_BGR2HSV);
//		for (int i=0; i<image.rows; i++) {
//			for (int j=0; j<image.cols; j++) {
//				cv::Vec3b RGB = image.at<cv::Vec3b>(i,j);
//				cv::Vec3b YCrCb = imageYCrCb.at<cv::Vec3b>(i,j);
//				cv::Vec3b Lab = imageLab.at<cv::Vec3b>(i,j);
//				cv::Vec3b HSV = imageHSV.at<cv::Vec3b>(i,j);
//				bool inlier = false;
//				if (LocalConfig::i0 && (RGB[0]   >= 0 		&&	RGB[0]   <= 62			&&
//							 RGB[1]   >= 17 		&&	RGB[1]   <= 89			&&
//							 RGB[2]   >= 52 		&&	RGB[2]   <= 136))
//					inlier = true;
//				if (LocalConfig::i1 && (YCrCb[0] >= 80		&&	YCrCb[0] <= 255		&&
//							 YCrCb[1] >= 135		&&	YCrCb[1] <= 180		&&
//							 YCrCb[2] >= 85		&&	YCrCb[2] <= 135))
//					inlier = true;
//				if (LocalConfig::i2 && (Lab[0]   >= 0 		&&	Lab[0]   <= 255			&&
//							 Lab[1]   >= 130 		&&	Lab[1]   <= 150			&&
//							 Lab[2]   >= 130 		&&	Lab[2]   <= 255))
//					inlier = true;
//				if (LocalConfig::i3 && (HSV[0]   >= 0			&&	HSV[0]   <= 28			&&
//							 HSV[1]   >= 69		&&	HSV[1]   <= 228		&&
//							 HSV[2]   >= 14		&&	HSV[2]   <= 255))
//					inlier = true;
//				if (!inlier)
//					image.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
//			}
//		}
//
//    cv_bridge::CvImagePtr cv_ptr1(new cv_bridge::CvImage);
//		cv_ptr1->header = msg_in.header;
//		cv_ptr1->encoding = sensor_msgs::image_encodings::BGR8;
//		cv_ptr1->image = image;
//		m_pubImg1.publish(cv_ptr1->toImageMsg());
//
//		cv::erode(image, image, cv::Mat(), cv::Point(-1, -1), 2); //2
//		cv::dilate(image, image, cv::Mat(), cv::Point(-1, -1), 2); //15
////		cv::erode(image, image, cv::Mat(), cv::Point(-1, -1), LocalConfig::i2); //15
////		cv::dilate(image, image, cv::Mat(), cv::Point(-1, -1), LocalConfig::i3);
//
//		cv_bridge::CvImagePtr cv_ptr2(new cv_bridge::CvImage);
//		cv_ptr2->header = msg_in.header;
//		cv_ptr2->encoding = sensor_msgs::image_encodings::BGR8;
//		cv_ptr2->image = image;
//		m_pubImg2.publish(cv_ptr2->toImageMsg());
//
//		cv_bridge::CvImagePtr cv_ptr3(new cv_bridge::CvImage);
//		cv_ptr3->header = msg_in.header;
//		cv_ptr3->encoding = sensor_msgs::image_encodings::BGR8;
//		cv_ptr3->image = imageHSV;
//		m_pubImg3.publish(cv_ptr3->toImageMsg());

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
    m_mins(0) += 0.03;
    m_mins(1) += 0.03;
    m_maxes(0) -= 0.03;
    m_maxes(1) -= 0.03;
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
    m_pubCloud(nh.advertise<sensor_msgs::PointCloud2>(outputNS+"/points",5)),
    m_pubCloudBorder(nh.advertise<sensor_msgs::PointCloud2>(outputNS+"/pointsBorder",5)),
    m_pubCloudComp(nh.advertise<sensor_msgs::PointCloud2>(outputNS+"/pointsComp",5)),
    m_polyPub(nh.advertise<geometry_msgs::PolygonStamped>(outputNS+"/polygon",5)),
    m_sub(nh.subscribe(LocalConfig::inputTopic, 1, &PreprocessorNode::callback, this)),
  	m_pubImg1(nh.advertise<sensor_msgs::Image>(outputNS+"/image1",5)),
		m_pubImg2(nh.advertise<sensor_msgs::Image>(outputNS+"/image2",5)),
		m_pubImg3(nh.advertise<sensor_msgs::Image>(outputNS+"/image3",5)),
		m_mins(-10,-10,-10),
		m_maxes(10,10,10),
		m_transform(toBulletTransform(Affine3f::Identity()))
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

  PreprocessorNode tp(nh);
  ros::spin();
}
