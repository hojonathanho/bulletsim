#include "utils/my_exceptions.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
#include "get_table2.h"
#include "get_chessboard_pose.h"
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
  static std::string outputTopic;
  static int imageCalibration;
  static float squareSize;
  static int chessBoardWidth;
  static int chessBoardHeight;
  static int SVDCalibration;
  static float minCorrFraction;
  static int ICPCalibration;
  static int maxIterations;
  static int RANSACIterations;
  static float RANSACOutlierRejectionThreshold;
  static float maxCorrespondenceDistance;
  static float transformationEpsilon;
  static float euclideanFitnessEpsilon;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("inputTopic1", &inputTopic1, "input topic1"));
    params.push_back(new Parameter<string>("inputTopic2", &inputTopic2, "input topic2"));
    params.push_back(new Parameter<string>("outputTopic", &outputTopic, "output topic"));
    params.push_back(new Parameter<int>("imageCalibration", &imageCalibration, "0 means that image calibration is NOT run"));
    params.push_back(new Parameter<float>("squareSize", &squareSize, "the length (in meters) of the sides of the squares (if imageCalibration!=0)"));
    params.push_back(new Parameter<int>("chessBoardWidth", &chessBoardWidth, "number of inner corners along the width of the chess board (if imageCalibration!=0 or SVDCalibration!=0)"));
    params.push_back(new Parameter<int>("chessBoardHeight", &chessBoardHeight, "number of inner corners along the height of the chess board (if imageCalibration!=0 or SVDCalibration!=0)"));
    params.push_back(new Parameter<int>("SVDCalibration", &SVDCalibration, "0 means that SVD calibration is NOT run. 2 means that chess board corners points are published into outputTopic/corners1 and outputTopic/corners2."));
    params.push_back(new Parameter<float>("minCorrFraction", &minCorrFraction, "minimum number of valid checker board corner correspondences (i.e. minCorrFraction*chessBoardWidth*chessBoardHeight) in order to run SVD calibration (if SVDCalibration!=0)"));
    params.push_back(new Parameter<int>("ICPCalibration", &ICPCalibration, "0 means that ICP calibration is NOT run"));
    params.push_back(new Parameter<int>("maxIterations", &maxIterations, "maximum number of iterations the internal optimization should run for (if ICPCalibration!=0)"));
    params.push_back(new Parameter<int>("RANSACIterations", &RANSACIterations, "the number of iterations RANSAC should run for (if ICPCalibration!=0)"));
    params.push_back(new Parameter<float>("RANSACOutlierRejectionThreshold", &RANSACOutlierRejectionThreshold, "the inlier distance threshold for the internal RANSAC outlier rejection loop (if ICPCalibration!=0)"));
    params.push_back(new Parameter<float>("maxCorrespondenceDistance", &maxCorrespondenceDistance, "the maximum distance threshold between two correspondent points in source <-> target (if ICPCalibration!=0)"));
    params.push_back(new Parameter<float>("transformationEpsilon", &transformationEpsilon, "the transformation epsilon (maximum allowable difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution (if ICPCalibration!=0)"));
    params.push_back(new Parameter<float>("euclideanFitnessEpsilon", &euclideanFitnessEpsilon, "the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the algorithm is considered to have converged (if ICPCalibration!=0)"));
  }
};

string LocalConfig::inputTopic1 = "/kinect1/depth_registered/points";
string LocalConfig::inputTopic2 = "/kinect2/depth_registered/points";
string LocalConfig::outputTopic = "/merger";
int LocalConfig::imageCalibration = 0;
float LocalConfig::squareSize = 0.041267;
int LocalConfig::chessBoardWidth = 6;
int LocalConfig::chessBoardHeight = 9;
int LocalConfig::SVDCalibration = 1;
float LocalConfig::minCorrFraction = 0.6;
int LocalConfig::ICPCalibration = 0;
int LocalConfig::maxIterations = 100;
int LocalConfig::RANSACIterations = 10;
float LocalConfig::RANSACOutlierRejectionThreshold = 0.005;
float LocalConfig::maxCorrespondenceDistance = 100;
float LocalConfig::transformationEpsilon = 0;
float LocalConfig::euclideanFitnessEpsilon = 0.0001;

typedef boost::shared_ptr< ::sensor_msgs::Image const> ImageConstPtr;

bool table_init = false;
bool image_calib_init = false;
bool svd_calib_init = false;
bool icp_calib_init = false;
Matrix4f transform_diff = Matrix4f::Identity();
btTransform ground_transform;
geometry_msgs::Polygon ground_poly;

boost::shared_ptr<ros::Publisher> cloudPub;
boost::shared_ptr<ros::Publisher> corners1Pub;
boost::shared_ptr<ros::Publisher> corners2Pub;
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

	ground_transform.setBasis(btMatrix3x3(xax(0),yax(0),zax(0),
										  xax(1),yax(1),zax(1),
										  xax(2),yax(2),zax(2)));
	ground_transform.setOrigin(btVector3(corners(0,0), corners(0,1), corners(0,2)));

	ground_poly.points = toROSPoints32(toBulletVectors(corners));

	table_init = true;
}

void SVDCalibration(ColorCloudPtr cloud1, ColorCloudPtr cloud2) {
	ColorCloudPtr cloud1_corners = chessBoardCorners(cloud1, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight);
	ColorCloudPtr cloud2_corners = chessBoardCorners(cloud2, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight);
	int nAllPoints = LocalConfig::chessBoardWidth * LocalConfig::chessBoardHeight;
	if ((cloud1_corners->size() == nAllPoints) &&
		(cloud2_corners->size() == nAllPoints)) {
		//Filter out the bad points from both point clouds
		vector<int> badPoints;
		for (int i=0; i<cloud1_corners->size(); i++) {
			if (!pointIsFinite(cloud1_corners->at(i)) || !pointIsFinite(cloud2_corners->at(i)))
				badPoints.push_back(i);
		}
		for (int i=(badPoints.size()-1); i>=0; i--) {
			//ROS_INFO("bad points %d", badPoints[i]);
			cloud1_corners->erase(cloud1_corners->begin() + badPoints[i]);
			cloud2_corners->erase(cloud2_corners->begin() + badPoints[i]);
		}

		int minCorr = LocalConfig::minCorrFraction * nAllPoints;
		if ((cloud1_corners->size()>minCorr) && (cloud2_corners->size()>minCorr)) {
			vector<int> indices;
			for (int i=0; i<cloud1_corners->size(); i++)
				indices.push_back(i);
			pcl::registration::TransformationEstimationSVD<ColorPoint, ColorPoint> estimation_svd;
			Matrix4f transform;
			estimation_svd.estimateRigidTransformation(*cloud2_corners, indices, *cloud1_corners, indices, transform_diff);

			svd_calib_init = true;
			ROS_INFO("SVD calibration succeeded with %d/%d points", (int) cloud1_corners->size(), nAllPoints);
		} else {
			ROS_WARN("SVD calibration failed. Only %d of a minimum of %d correspondences found.", (int) cloud1_corners->size(), minCorr);
		}
	} else {
		ROS_WARN("SVD calibration failed. Make sure both cameras sees the chess board. First camera sees %d/%d. Second camera sees %d/%d.", (int) cloud1_corners->size(),
																																			nAllPoints,
																																			(int) cloud2_corners->size(),
																																			nAllPoints);
	}
}

void ICPCalibration(ColorCloudPtr cloud1, ColorCloudPtr cloud2) {
	pcl::IterativeClosestPoint<ColorPoint, ColorPoint> icp;
	icp.setInputCloud(downsampleCloud(cloud2, 0.2));
	icp.setInputTarget(downsampleCloud(cloud1, 0.2));
	icp.setMaximumIterations(LocalConfig::maxIterations);
	icp.setRANSACIterations(LocalConfig::RANSACIterations);
	icp.setRANSACOutlierRejectionThreshold(LocalConfig::RANSACOutlierRejectionThreshold);
	icp.setMaxCorrespondenceDistance(LocalConfig::maxCorrespondenceDistance);
	icp.setTransformationEpsilon(LocalConfig::transformationEpsilon);
	icp.setEuclideanFitnessEpsilon(LocalConfig::euclideanFitnessEpsilon);
	ROS_INFO("ICP aligning of the two clouds...");
	ColorCloud Final;
	icp.align(Final, transform_diff);
	if (icp.hasConverged()) {
		transform_diff = icp.getFinalTransformation();
		icp_calib_init = true;
		ROS_INFO("ICP calibration succeeded with fitness score of %.4f", icp.getFitnessScore());
	} else {
		ROS_WARN("ICP calibration failed with fitness score of %.4f", icp.getFitnessScore());
	}
}

void imageCalibration(ColorCloudPtr cloud1, ColorCloudPtr cloud2) {
	cv::Mat image1 = toCVMatImage(cloud1);
	cv::Mat image2 = toCVMatImage(cloud2);

	double CX = 320-.5;
	double CY = 240-.5;
	double F = 525;
	Matrix3f cam_matrix;
	cam_matrix(0,0) = cam_matrix(1,1) = F;
	cam_matrix(0,2) = CX;
	cam_matrix(1,2) = CY;

	Matrix4f transform1, transform2;
	if (get_chessboard_pose(image1, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight, LocalConfig::squareSize, cam_matrix, transform1) &&
		get_chessboard_pose(image2, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight, LocalConfig::squareSize, cam_matrix, transform2)) {
		transform_diff = transform1*transform2.inverse();
		image_calib_init = true;
		ROS_INFO("Image calibration suceeded");
	} else {
		ROS_WARN("Image calibration failed. Make sure both cameras sees the chess board.");
	}
}

void callback(const sensor_msgs::PointCloud2ConstPtr& msg_in1, const sensor_msgs::PointCloud2ConstPtr& msg_in2) {
	ColorCloudPtr cloud_in1(new ColorCloud());
	ColorCloudPtr cloud_in2(new ColorCloud());
	pcl::fromROSMsg(*msg_in1, *cloud_in1);
	pcl::fromROSMsg(*msg_in2, *cloud_in2);

	if (LocalConfig::imageCalibration && !image_calib_init)
		imageCalibration(cloud_in1, cloud_in2);
	if (LocalConfig::SVDCalibration && !svd_calib_init)
		SVDCalibration(cloud_in1, cloud_in2);
	if (LocalConfig::ICPCalibration && !icp_calib_init)
		ICPCalibration(cloud_in1, cloud_in2);
	if (!table_init)
		initTable(cloud_in1);

	ColorCloudPtr cloud_out(new ColorCloud(*cloud_in1));
	pcl::transformPointCloud(*cloud_in2.get(), *cloud_in2.get(), transform_diff);
	ColorCloud::iterator it;
	for (it = cloud_in2->begin(); it < cloud_in2->end(); it++)
		cloud_out->push_back(*it);
	//pcl::io::savePCDFile("/home/alex/rll/test/data/merged.pcd", *cloud_out);

	sensor_msgs::PointCloud2 msg_out;
	pcl::toROSMsg(*cloud_out, msg_out);
	msg_out.header = msg_in1->header;
	cloudPub->publish(msg_out);

	if (LocalConfig::SVDCalibration == 2) {
		sensor_msgs::PointCloud2 msg_corners1;
		pcl::toROSMsg(*chessBoardCorners(cloud_in1, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight), msg_corners1);
		msg_corners1.header = msg_in1->header;
		corners1Pub->publish(msg_corners1);

		sensor_msgs::PointCloud2 msg_corners2;
		pcl::toROSMsg(*chessBoardCorners(cloud_in2, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight), msg_corners2);
		msg_corners2.header = msg_in1->header;
		corners2Pub->publish(msg_corners2);
	}

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

	cloudPub.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>(LocalConfig::outputTopic+"/points",5)));
	if (LocalConfig::SVDCalibration == 2) {
		corners1Pub.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>(LocalConfig::outputTopic+"/corners1",5)));
		corners2Pub.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>(LocalConfig::outputTopic+"/corners2",5)));
	}
	polyPub.reset(new ros::Publisher(nh.advertise<geometry_msgs::PolygonStamped>(LocalConfig::outputTopic+"/polygon",5)));
	broadcaster.reset(new tf::TransformBroadcaster());

	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1Sub(nh, LocalConfig::inputTopic1, 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2Sub(nh, LocalConfig::inputTopic2, 1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> CloudSyncPolicy;
	message_filters::Synchronizer<CloudSyncPolicy> cloudSync(CloudSyncPolicy(30), cloud1Sub, cloud2Sub);
	cloudSync.registerCallback(boost::bind(&callback,_1,_2));

	ros::spin();
}
