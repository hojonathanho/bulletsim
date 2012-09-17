#include <cmath>
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <cv_bridge/cv_bridge.h>
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
#include "get_chessboard_pose.h"
#include "utils/my_exceptions.h"
#include "utils/config.h"
#include "utils/conversions.h"
#include "utils_ros.h"

using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
  static std::vector<std::string> cameraTopics;
  static std::string outputTopic;
  static int calibrationType;
  static bool calibrateOnce;
  static float squareSize;
  static int chessBoardWidth;
  static int chessBoardHeight;
  static int tfParent;
  static bool saveTransform;

  LocalConfig() : Config() {
    params.push_back(new Parameter<std::vector<std::string> >("cameraTopics", &cameraTopics, "camera base topics. there should be at least two."));
    params.push_back(new Parameter<string>("outputTopic", &outputTopic, "output topic for the merged cloud of the first two cameraTopics"));
    params.push_back(new Parameter<int>("calibrationType", &calibrationType, "0: Load from file calibration \n1:SVD calibration \n2: image calibration"));
    params.push_back(new Parameter<bool>("calibrateOnce", &calibrateOnce, "the camera is calibrated once or continuously"));
    params.push_back(new Parameter<float>("squareSize", &squareSize, "the length (in meters) of the sides of the squares (if calibrationType != 0)"));
    params.push_back(new Parameter<int>("chessBoardWidth", &chessBoardWidth, "number of inner corners along the width of the chess board (if calibrationType != 0)"));
    params.push_back(new Parameter<int>("chessBoardHeight", &chessBoardHeight, "number of inner corners along the height of the chess board (if calibrationType != 0)"));
    params.push_back(new Parameter<int>("tfParent", &tfParent, "which camera should be the parent for the chessboard frame. defaults to -1, meaning that the chessboard is the parent of everything"));
    params.push_back(new Parameter<bool>("saveTransform", &saveTransform, "save chessboard <-> camera transforms to file"));
  }
};

std::vector<std::string> LocalConfig::cameraTopics = boost::assign::list_of("/kinect1/depth_registered/points")("/kinect2/depth_registered/points");
string LocalConfig::outputTopic = "/kinect_merged/points";
int LocalConfig::calibrationType = 0;
bool LocalConfig::calibrateOnce = true;
float LocalConfig::squareSize = 0.0272;
int LocalConfig::chessBoardWidth = 6;
int LocalConfig::chessBoardHeight = 7;
int LocalConfig::tfParent = -1;
bool LocalConfig::saveTransform = true;

double CX = 320-.5;
double CY = 240-.5;
double F = 525;
Matrix3f cam_matrix = Matrix3f::Identity();

vector<bool> calib_inits;
vector<Matrix4f> transforms;

boost::shared_ptr<tf::TransformBroadcaster> broadcaster;
boost::shared_ptr<tf::TransformListener> listener;

boost::shared_ptr<ros::Publisher> cloudPub;
sensor_msgs::PointCloud2 msg_out;

void mergeCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in1, const sensor_msgs::PointCloud2ConstPtr& msg_in2) {
	ColorCloudPtr cloud_in1(new ColorCloud());
	ColorCloudPtr cloud_in2(new ColorCloud());
	pcl::fromROSMsg(*msg_in1, *cloud_in1);
	pcl::fromROSMsg(*msg_in2, *cloud_in2);

	pcl::transformPointCloud(*cloud_in1.get(), *cloud_in1.get(), transforms[0]);
	pcl::transformPointCloud(*cloud_in2.get(), *cloud_in2.get(), transforms[1]);
	ColorCloudPtr cloud_out(new ColorCloud(*cloud_in1 + *cloud_in2));

	pcl::toROSMsg(*cloud_out, msg_out);
	msg_out.header.seq++;
	msg_out.header.stamp = ros::Time::now();
	msg_out.header.frame_id = "chess_board";
	cloudPub->publish(msg_out);
}

void updateTransformCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in, int i) {
	if (LocalConfig::calibrateOnce && calib_inits[i]) return;

	ColorCloudPtr cloud_in(new ColorCloud());
	pcl::fromROSMsg(*msg_in, *cloud_in);

	switch (LocalConfig::calibrationType) {
	// Load from file calibration
	case 0:
	{
		if (loadTransform(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms" + string(msg_in->header.frame_id), transforms[i])) {
			calib_inits[i] = true;
			ROS_INFO("Load from file calibration on camera %d succeeded.", i);
		} else {
			ROS_WARN("Load from file calibration on camera %d failed. Make sure the files exist.", i);
		}
		break;
	}

	// SVD calibration
	case 1:
	{
		int found_corners = getChessBoardPose(cloud_in, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight, LocalConfig::squareSize, transforms[i]);
		if (found_corners) {
			calib_inits[i] = true;
			ROS_INFO("SVD calibration on camera %d succeeded %d/%d corners.", i, found_corners, LocalConfig::chessBoardWidth*LocalConfig::chessBoardHeight);
		} else {
			ROS_WARN("SVD calibration on camera %d failed. Camera %d sees %d/%d corners.", i, i, found_corners, LocalConfig::chessBoardWidth*LocalConfig::chessBoardHeight);
		}
		break;
	}

	// Image calibration
	case 2:
	{
		cv::Mat image = toCVMatImage(cloud_in);
		if (get_chessboard_pose(image, LocalConfig::chessBoardWidth, LocalConfig::chessBoardHeight, LocalConfig::squareSize, cam_matrix, transforms[i])) {
			calib_inits[i] = true;
			ROS_INFO("Image calibration on camera %d succeeded.", i);
		} else {
			ROS_WARN("Image calibration on camera %d failed. Make sure camera %d sees the chess board.", i, i);
		}
		break;
	}

	// Invalid calibration type
	default:
		ROS_WARN("Calibration type %d is invalid.", LocalConfig::calibrationType);
	}

	if (calib_inits[i] && (LocalConfig::calibrationType!=0) && (LocalConfig::saveTransform))
		saveTransform(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/transforms" + string(msg_in->header.frame_id), transforms[i]);
}

void broadcastTransformCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in, int i) {
	try {
	  if (i == LocalConfig::tfParent) {
	    string link;
	    listener->getParent(msg_in->header.frame_id, ros::Time(0), link);
        listener->getParent(link, ros::Time(0), link);
        tf::StampedTransform tfTransform;
        listener->lookupTransform (link, msg_in->header.frame_id, ros::Time(0), tfTransform);
	    broadcaster->sendTransform(tf::StampedTransform(tfTransform.asBt() * toBulletTransform((Affine3f)transforms[i]).inverse(), ros::Time::now(), link,  "chess_board"));
	  }
	  else {
        broadcastKinectTransform(toBulletTransform((Affine3f) transforms[i]), msg_in->header.frame_id, "chess_board", *broadcaster, *listener);
	  }
	} catch (...) {
		ROS_WARN("Caught an exception from broadcastKinectTransform. Skipping...");
	}
}


int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	cam_matrix << F, 0, CX,
		            0, F, CY,
		            0, 0, 1;

	int nCameras = LocalConfig::cameraTopics.size();
	calib_inits.assign(nCameras, false);
	transforms.assign(nCameras, Matrix4f::Identity());

	ros::init(argc, argv,"merger");
	ros::NodeHandle nh;

	broadcaster.reset(new tf::TransformBroadcaster());
	listener.reset(new tf::TransformListener());

	cloudPub.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>(LocalConfig::outputTopic,5)));
	//msg_out.header.seq = 0;

	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1Sub(nh, LocalConfig::cameraTopics[0], 20);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2Sub(nh, LocalConfig::cameraTopics[1], 20);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> CloudSyncPolicy;
	message_filters::Synchronizer<CloudSyncPolicy>* cloudSync = new message_filters::Synchronizer<CloudSyncPolicy>(CloudSyncPolicy(30), cloud1Sub, cloud2Sub);
	cloudSync->registerCallback(boost::bind(&mergeCloudCallback,_1,_2));

	vector<ros::Subscriber> cloudsSubUpdate(nCameras);
	vector<ros::Subscriber> cloudsSubBroadcast(nCameras);
	for (int i=0; i<nCameras; i++) {
		cloudsSubUpdate[i] = nh.subscribe<sensor_msgs::PointCloud2>(LocalConfig::cameraTopics[i], 1, boost::bind(updateTransformCallback,_1,i));
		cloudsSubBroadcast[i] = nh.subscribe<sensor_msgs::PointCloud2>(LocalConfig::cameraTopics[i], 1, boost::bind(broadcastTransformCallback,_1,i));
	}

	if (LocalConfig::calibrateOnce) {
		ros::spin();
	} else {
		ros::MultiThreadedSpinner spinner(4);
		spinner.spin();
	}
}
