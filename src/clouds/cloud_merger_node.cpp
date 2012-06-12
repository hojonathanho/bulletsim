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
  static std::string inputImageTopic1;
  static std::string inputImageTopic2;
  static float zClipLow;
  static float zClipHigh;
  static int checkerBoardWidth;
  static int checkerBoardHeight;
  static float squareSize;
  static int imageCalibration;
  static int SVDCalibration;
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
    params.push_back(new Parameter<string>("inputImageTopic1", &inputImageTopic1, "input image topic1"));
    params.push_back(new Parameter<string>("inputImageTopic2", &inputImageTopic2, "input image topic2"));
    params.push_back(new Parameter<float>("zClipLow", &zClipLow, "clip points that are less than this much above table"));
    params.push_back(new Parameter<float>("zClipHigh", &zClipHigh, "clip points that are more than this much above table"));
    params.push_back(new Parameter<int>("checkerBoardWidth", &checkerBoardWidth, "number of inner corners along the width of the checker board"));
    params.push_back(new Parameter<int>("checkerBoardHeight", &checkerBoardHeight, "number of inner corners along the height of the checker board"));
    params.push_back(new Parameter<float>("squareSize", &squareSize, "the length (in meters) of the sides of the squares"));
    params.push_back(new Parameter<int>("imageCalibration", &imageCalibration, "0 means that image calibration is NOT run"));
    params.push_back(new Parameter<int>("SVDCalibration", &SVDCalibration, "0 means that SVD calibration is NOT run"));
    params.push_back(new Parameter<int>("ICPCalibration", &ICPCalibration, "0 means that ICP calibration is NOT run"));
    params.push_back(new Parameter<int>("maxIterations", &maxIterations, "maximum number of iterations the internal optimization should run for"));
    params.push_back(new Parameter<int>("RANSACIterations", &RANSACIterations, "the number of iterations RANSAC should run for"));
    params.push_back(new Parameter<float>("RANSACOutlierRejectionThreshold", &RANSACOutlierRejectionThreshold, "the inlier distance threshold for the internal RANSAC outlier rejection loop"));
    params.push_back(new Parameter<float>("maxCorrespondenceDistance", &maxCorrespondenceDistance, "the maximum distance threshold between two correspondent points in source <-> target"));
    params.push_back(new Parameter<float>("transformationEpsilon", &transformationEpsilon, "the transformation epsilon (maximum allowable difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution"));
    params.push_back(new Parameter<float>("euclideanFitnessEpsilon", &euclideanFitnessEpsilon, "the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the algorithm is considered to have converged"));
  }
};

string LocalConfig::inputTopic1 = "/kinect1/depth_registered/points";
string LocalConfig::inputTopic2 = "/kinect2/depth_registered/points";
string LocalConfig::inputImageTopic1 = "/kinect1/rgb/image_color";
string LocalConfig::inputImageTopic2 = "/kinect2/rgb/image_color";
float LocalConfig::zClipLow = .0025;
float LocalConfig::zClipHigh = 1000;
int LocalConfig::checkerBoardWidth = 6;
int LocalConfig::checkerBoardHeight = 9;
float LocalConfig::squareSize = 0.041267;
int LocalConfig::imageCalibration = 0;
int LocalConfig::SVDCalibration = 1;
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

	table_init = true;
}

void SVDCalibration(ColorCloudPtr cloud1, ColorCloudPtr cloud2) {
	ColorCloudPtr cloud1_corners = checkerBoardCorners(cloud1, LocalConfig::checkerBoardWidth, LocalConfig::checkerBoardHeight);
	ColorCloudPtr cloud2_corners = checkerBoardCorners(cloud2, LocalConfig::checkerBoardWidth, LocalConfig::checkerBoardHeight);
	ROS_INFO("corner sizes %d %d", (int) cloud1_corners->size(), (int) cloud2_corners->size());
	if ((cloud1_corners->size() == LocalConfig::checkerBoardWidth * LocalConfig::checkerBoardHeight) &&
		(cloud2_corners->size() == LocalConfig::checkerBoardWidth * LocalConfig::checkerBoardHeight)) {
		//Filter out the bad points from both point clouds
		MatrixXf corners1_mat = toEigenMatrix(cloud1_corners);
		MatrixXf corners2_mat = toEigenMatrix(cloud2_corners);
		vector<int> badPoints;
		for (int i=0; i<cloud1_corners->size(); i++) {
			if (isnan(corners1_mat(i,0)) || isnan(corners2_mat(i,0)) ||
				isnan(corners1_mat(i,1)) || isnan(corners2_mat(i,1)) ||
				isnan(corners1_mat(i,2)) || isnan(corners2_mat(i,2)))
				badPoints.push_back(i);
		}
		for (int i=(badPoints.size()-1); i>=0; i--) {
			ROS_INFO("bad points %d", badPoints[i]);
			cloud1_corners->erase(cloud1_corners->begin() + badPoints[i]);
			cloud2_corners->erase(cloud2_corners->begin() + badPoints[i]);
		}

		MatrixXf xyz = toEigenMatrix(cloud1_corners);
		for(int i=0; i<cloud1_corners->size(); i++)
			ROS_INFO("cloud1_corners[%d] %.4f %.4f %.4f", i, xyz(i,0), xyz(i,1), xyz(i,2));

		vector<int> indices;
		for (int i=0; i<cloud1_corners->size(); i++)
			indices.push_back(i);
		pcl::registration::TransformationEstimationSVD<ColorPoint, ColorPoint> estimation_svd;
		Matrix4f transform;
		estimation_svd.estimateRigidTransformation(*cloud2_corners, indices, *cloud1_corners, indices, transform);
		bool valid_transform = true;
		for (int i=0; i<4; i++)
			for (int j=0; j<4; j++)
				if (isnan(transform(j,i))) {
					valid_transform = false;
					break;
				}
		int nAllPoints = LocalConfig::checkerBoardWidth * LocalConfig::checkerBoardHeight;
		if ((cloud1_corners->size()>(0.5*nAllPoints)) && (cloud2_corners->size()>(0.5*nAllPoints)) && valid_transform) {
			transform_diff = transform;
			svd_calib_init = true;
			ROS_INFO("transform_diff %.4f %.4f %.4f", transform_diff(0,3), transform_diff(1,3), transform_diff(2,3));
			ROS_INFO("SVD calibration succeeded with %d %d points", (int) cloud1_corners->size(), (int) cloud2_corners->size());
		} else {
			ROS_WARN("SVD calibration failed. SVD transformation estimation failed.");
		}
	} else {
		ROS_WARN("SVD calibration failed. Make sure both cameras sees the checker board. First camera sees %d/%d. Second camera sees %d/%d.", (int) cloud1_corners->size(),
																																			  LocalConfig::checkerBoardWidth * LocalConfig::checkerBoardHeight,
																																			  (int) cloud2_corners->size(),
																																			  LocalConfig::checkerBoardWidth * LocalConfig::checkerBoardHeight);
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
	cv::Mat image1 = cloud2Image(cloud1);
	cv::Mat image2 = cloud2Image(cloud2);

	double CX = 320-.5;
	double CY = 240-.5;
	double F = 525;
	Matrix3f cam_matrix;
	cam_matrix(0,0) = cam_matrix(1,1) = F;
	cam_matrix(0,2) = CX;
	cam_matrix(1,2) = CY;

	Matrix4f transform1, transform2;
	if (get_chessboard_pose(image1, LocalConfig::checkerBoardWidth, LocalConfig::checkerBoardHeight, LocalConfig::squareSize, cam_matrix, transform1) &&
		get_chessboard_pose(image2, LocalConfig::checkerBoardWidth, LocalConfig::checkerBoardHeight, LocalConfig::squareSize, cam_matrix, transform2)) {
		transform_diff = transform1*transform2.inverse();
		image_calib_init = true;
		ROS_INFO("Image calibration suceeded");
	} else {
		ROS_WARN("Image calibration failed. Make sure both cameras sees the checker board.");
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

	sensor_msgs::PointCloud2 msg_corners1;
	pcl::toROSMsg(*checkerBoardCorners(cloud_in1, LocalConfig::checkerBoardWidth, LocalConfig::checkerBoardHeight), msg_corners1);
	msg_corners1.header = msg_in1->header;
	corners1Pub->publish(msg_corners1);

	sensor_msgs::PointCloud2 msg_corners2;
	pcl::toROSMsg(*checkerBoardCorners(cloud_in2, LocalConfig::checkerBoardWidth, LocalConfig::checkerBoardHeight), msg_corners2);
	msg_corners2.header = msg_in1->header;
	corners2Pub->publish(msg_corners2);

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
	corners1Pub.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>("/merger/corners1",5)));
	corners2Pub.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>("/merger/corners2",5)));
	polyPub.reset(new ros::Publisher(nh.advertise<geometry_msgs::PolygonStamped>("/merger/polygon",5)));
	broadcaster.reset(new tf::TransformBroadcaster());

	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1Sub(nh, LocalConfig::inputTopic1, 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2Sub(nh, LocalConfig::inputTopic2, 1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> CloudSyncPolicy;
	message_filters::Synchronizer<CloudSyncPolicy> cloudSync(CloudSyncPolicy(30), cloud1Sub, cloud2Sub);
	cloudSync.registerCallback(boost::bind(&callback,_1,_2));

	ros::spin();
}
