#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_keypoint3D.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
#include "utils/config.h"
#include "utils/conversions.h"
#include <boost/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
  static std::string inputTopic;
  static std::string outputTopic;
  static float angularResolution;
  static float supportSize;
  static int opt;
  static float minScale;
  static int nrOctaves;
  static int nrOctavesPerScale;
  static float minContrast;
  static float harrisRadius;
  static float cannyThresh1;
  static float cannyThresh2;
  static int cannyAptSize;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("inputTopic", &inputTopic, "input topic"));
    params.push_back(new Parameter<string>("outputTopic", &outputTopic, "output topic"));
    params.push_back(new Parameter<float>("angularResolution", &angularResolution, "Resolution of the range image"));
    params.push_back(new Parameter<float>("supportSize", &supportSize, "Support size for the interest points (diameter of the used sphere"));
    params.push_back(new Parameter<int>("opt", &opt, "opt"));
    params.push_back(new Parameter<float>("minScale", &minScale, ""));
    params.push_back(new Parameter<int>("nrOctaves", &nrOctaves, ""));
    params.push_back(new Parameter<int>("nrOctavesPerScale", &nrOctavesPerScale, ""));
    params.push_back(new Parameter<float>("minContrast", &minContrast, ""));
    params.push_back(new Parameter<float>("harrisRadius", &harrisRadius, ""));
    params.push_back(new Parameter<float>("cannyThresh1", &cannyThresh1, ""));
    params.push_back(new Parameter<float>("cannyThresh2", &cannyThresh2, ""));
    params.push_back(new Parameter<int>("cannyAptSize", &cannyAptSize, ""));
  }
};

string LocalConfig::inputTopic = "/kinect1/depth_registered";
string LocalConfig::outputTopic = "/test";
float LocalConfig::angularResolution = 0.5f;
float LocalConfig::supportSize = 0.2f;
int LocalConfig::opt = 0;
float LocalConfig::minScale = 0.01;
int LocalConfig::nrOctaves = 3;
int LocalConfig::nrOctavesPerScale = 3;
float LocalConfig::minContrast = 10.0;
float LocalConfig::harrisRadius = 0.01;
float LocalConfig::cannyThresh1 = 10;
float LocalConfig::cannyThresh2 = 100;
int LocalConfig::cannyAptSize = 3;

boost::shared_ptr<ros::Publisher> cloudBorderPub;
boost::shared_ptr<ros::Publisher> cloudNarfPub;
boost::shared_ptr<ros::Publisher> cloudPlanarBorderPub;
boost::shared_ptr<ros::Publisher> cloudPlanarNarfPub;
boost::shared_ptr<ros::Publisher> imagePub;

ColorCloudPtr getBordersCloud(pcl::RangeImage range_image) {
	pcl::PointCloud<pcl::BorderDescription> border_descriptions;
	pcl::RangeImageBorderExtractor border_extractor (&range_image);
	border_extractor.compute (border_descriptions);

	pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
																						veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
																						shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
	pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
																			& veil_points = * veil_points_ptr,
																			& shadow_points = *shadow_points_ptr;
	for (int y=0; y< (int)range_image.height; ++y)
	{
		for (int x=0; x< (int)range_image.width; ++x)
		{
			if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
				border_points.points.push_back (range_image.points[y*range_image.width + x]);
			if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
				veil_points.points.push_back (range_image.points[y*range_image.width + x]);
			if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
				shadow_points.points.push_back (range_image.points[y*range_image.width + x]);
		}
	}

	ColorCloudPtr cloud_out(new ColorCloud());
	for (int i=0; i<border_points.size(); i++) {
		ColorPoint pt(0,255,0);
		pt.x = border_points.at(i).x;
		pt.y = border_points.at(i).y;
		pt.z = border_points.at(i).z;
		cloud_out->push_back(pt);
	}
	for (int i=0; i<veil_points.size(); i++) {
		ColorPoint pt(255,0,0);
		pt.x = veil_points.at(i).x;
		pt.y = veil_points.at(i).y;
		pt.z = veil_points.at(i).z;
		cloud_out->push_back(pt);
	}
	for (int i=0; i<shadow_points.size(); i++) {
		ColorPoint pt(0,255,255);
		pt.x = shadow_points.at(i).x;
		pt.y = shadow_points.at(i).y;
		pt.z = shadow_points.at(i).z;
		cloud_out->push_back(pt);
	}

	return cloud_out;
}

ColorCloudPtr getNARFCloud(pcl::RangeImage range_image) {
	pcl::RangeImageBorderExtractor range_image_border_extractor;
	pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
	narf_keypoint_detector.setRangeImage (&range_image);
	narf_keypoint_detector.getParameters ().support_size = LocalConfig::supportSize;
	//narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
	//narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

	pcl::PointCloud<int> keypoint_indices;
	narf_keypoint_detector.compute (keypoint_indices);

	ColorCloudPtr keypoints_ptr (new ColorCloud);
	ColorCloud& keypoints = *keypoints_ptr;
	keypoints.points.resize (keypoint_indices.points.size ());
	for (size_t i=0; i<keypoint_indices.points.size (); ++i)
		keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();

	return keypoints_ptr;
}

ColorCloudPtr getSIFTCloud (ColorCloudPtr &cloud_in)
{
	pcl::SIFTKeypoint<ColorPoint, pcl::PointWithScale> sift_detect;

	// Use a FLANN-based KdTree to perform neighborhood searches
	sift_detect.setSearchMethod (pcl::search::KdTree<ColorPoint>::Ptr (new pcl::search::KdTree<ColorPoint>));

  // Set the detection parameters
  sift_detect.setScales (LocalConfig::minScale, LocalConfig::nrOctaves, LocalConfig::nrOctavesPerScale);
  sift_detect.setMinimumContrast (LocalConfig::minContrast);

  // Set the input
  sift_detect.setInputCloud (cloud_in);

  // Detect the keypoints and store them in "keypoints_out"
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints (new pcl::PointCloud<pcl::PointWithScale>);
  sift_detect.compute (*keypoints);

  ColorCloudPtr cloud_out (new ColorCloud);
  for (size_t i=0; i<keypoints->size(); i++) {
  	ColorPoint pt(0,255,0);
  	pt.x = keypoints->at(i).x;
  	pt.y = keypoints->at(i).y;
  	pt.z = keypoints->at(i).z;
  	cloud_out->push_back(pt);
  }
  return cloud_out;
}

void callback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud) {
	ColorCloudPtr cloud_in(new ColorCloud());
	pcl::fromROSMsg(*msg_cloud, *cloud_in);

	Eigen::Affine3f scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (cloud_in->sensor_origin_[0], cloud_in->sensor_origin_[1], cloud_in->sensor_origin_[2])) * Eigen::Affine3f (cloud_in->sensor_orientation_);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	range_image.createFromPointCloud (*cloud_in, LocalConfig::angularResolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
																	 scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	range_image.integrateFarRanges (far_ranges);
	range_image.setUnseenToMaxRange ();

	/////////////////////////////////////////////////////////// BORDERS
	ColorCloudPtr cloud_out = getBordersCloud(range_image);

	sensor_msgs::PointCloud2 msg_border_cloud;
	pcl::toROSMsg(*cloud_out, msg_border_cloud);
	msg_border_cloud.header = msg_cloud->header;
	cloudBorderPub->publish(msg_border_cloud);

	/////////////////////////////////////////////////////////// NARF
	//ColorCloudPtr keypoints_ptr = getNARFCloud(range_image);
	ColorCloudPtr keypoints_ptr = getSIFTCloud(cloud_in);

	sensor_msgs::PointCloud2 msg_narf_cloud;
	pcl::toROSMsg(*keypoints_ptr, msg_narf_cloud);
	msg_narf_cloud.header = msg_cloud->header;
	cloudNarfPub->publish(msg_narf_cloud);
}

void callbackPlanar(const sensor_msgs::ImageConstPtr& msg_depth_image, const sensor_msgs::CameraInfoConstPtr& msg_camera_info) {
	boost::shared_ptr<pcl::RangeImagePlanar> range_image_planar_ptr (new pcl::RangeImagePlanar);
	pcl::RangeImagePlanar& range_image_planar = *range_image_planar_ptr;

	range_image_planar.setDepthImage(reinterpret_cast<const float*> (&msg_depth_image->data[0]),
			msg_depth_image->width, msg_depth_image->height,
			msg_camera_info->P[2],  msg_camera_info->P[6],
			msg_camera_info->P[0],  msg_camera_info->P[5], LocalConfig::angularResolution);

	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	range_image_planar.integrateFarRanges (far_ranges);
	range_image_planar.setUnseenToMaxRange ();

	/////////////////////////////////////////////////////////// BORDERS
	ColorCloudPtr cloud_out = getBordersCloud(range_image_planar);

	sensor_msgs::PointCloud2 msg_border_cloud;
	pcl::toROSMsg(*cloud_out, msg_border_cloud);
	msg_border_cloud.header = msg_depth_image->header;
	cloudPlanarBorderPub->publish(msg_border_cloud);

	/////////////////////////////////////////////////////////// NARF
	ColorCloudPtr keypoints_ptr = getNARFCloud(range_image_planar);

	sensor_msgs::PointCloud2 msg_narf_cloud;
	pcl::toROSMsg(*keypoints_ptr, msg_narf_cloud);
	msg_narf_cloud.header = msg_depth_image->header;
	cloudPlanarNarfPub->publish(msg_narf_cloud);
}

//./bin/cloud_test_node --inputTopic=/kinect1/rgb/age_color --cannyThresh1=200 --cannyThresh2=500 --cannyAptSize=5
//./bin/cloud_test_node --inputTopic=/kinect1/deptregistered/image --cannyThresh1=10 --cannyThresh2=30 --cannyAptSize=3
void callbackCanny(const sensor_msgs::ImageConstPtr& msg_img) {
  cv::Mat img = cv_bridge::toCvCopy(msg_img)->image;

  cv::Mat img_gray(img.rows, img.cols, CV_8UC1);
  if (img.type() == CV_8UC3) {
  	cvtColor(img, img, CV_BGR2Lab);
  	cvtColor(img, img_gray, CV_BGR2GRAY);
  } else if (img.type() == CV_32F)
  	cv::convertScaleAbs(img, img_gray, 256);

  cout << "type " <<  img_gray.type() << " " << CV_8UC1 << " " << CV_32F << endl;

  assert( img_gray.cols%2 == 0 && img_gray.rows%2 == 0);
  cv::Mat canny_img(img_gray.rows/2, img_gray.cols, img_gray.type());
//  IplImage* out = cvCreateImage( cvSize(img->width/2,img->height/2), img->depth, img->nChannels );
  cv::pyrDown(img_gray, canny_img);
  cv::Canny(canny_img, canny_img, LocalConfig::cannyThresh1, LocalConfig::cannyThresh2, LocalConfig::cannyAptSize);

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	cv_ptr->header = msg_img->header;
	cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
	cv_ptr->image = canny_img;
	imagePub->publish(cv_ptr->toImageMsg());
}

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	ros::init(argc, argv,"test");
	ros::NodeHandle nh;

	LocalConfig::angularResolution = pcl::deg2rad (LocalConfig::angularResolution);

	cloudBorderPub.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>(LocalConfig::outputTopic+"/cloud/border/points",5)));
	cloudNarfPub.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>(LocalConfig::outputTopic+"/cloud/narf/points",5)));
	cloudPlanarBorderPub.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>(LocalConfig::outputTopic+"/planar/border/points",5)));
	cloudPlanarNarfPub.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>(LocalConfig::outputTopic+"/planar/narf/points",5)));
	imagePub.reset(new ros::Publisher(nh.advertise<sensor_msgs::Image>(LocalConfig::outputTopic+"/image",5)));

	cout << "Publishing to" << endl;
	cout << "\t" << LocalConfig::outputTopic+"/cloud/border/points" << endl;
	cout << "\t" << LocalConfig::outputTopic+"/cloud/narf/points" << endl;
	cout << "\t" << LocalConfig::outputTopic+"/planar/border/points" << endl;
	cout << "\t" << LocalConfig::outputTopic+"/planar/narf/points" << endl;


	ros::Subscriber cloudSub = nh.subscribe(LocalConfig::inputTopic+"/points", 1, callback);
	ros::Subscriber imageSub = nh.subscribe(LocalConfig::inputTopic, 1, callbackCanny);

	message_filters::Subscriber<sensor_msgs::Image> depthSub(nh, LocalConfig::inputTopic+"/image_rect", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub(nh, LocalConfig::inputTopic+"/camera_info", 1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
	message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(30), depthSub, cameraInfoSub);
	sync.registerCallback(boost::bind(&callbackPlanar,_1,_2));

	cout << "Subscribed to" << endl;
	cout << "\t" << LocalConfig::inputTopic+"/points" << endl;
	cout << "\t" << LocalConfig::inputTopic+"/image_rect" << endl;
	cout << "\t" << LocalConfig::inputTopic+"/camera_info" << endl;

	ros::spin();
}
