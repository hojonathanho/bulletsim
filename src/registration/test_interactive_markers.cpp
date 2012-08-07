#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <utils/config.h>


struct MoveXYZ {
  visualization_msgs::Marker box_marker;
  visualization_msgs::InteractiveMarker int_marker;
  visualization_msgs::InteractiveMarkerControl box_control;
  MoveXYZ() {
	  printf("oshit\n");
  }
  MoveXYZ(std::string name) {
	  int_marker.header.frame_id = "/base_link";
	  int_marker.name = name;
	  int_marker.description = name;
	  int_marker.scale = .05;
	  box_marker.type = visualization_msgs::Marker::CUBE;

	  box_marker.scale.x = 0.02;
	  box_marker.scale.y = 0.02;
	  box_marker.scale.z = 0.02;
	  box_marker.color.r = 0.5;
	  box_marker.color.g = 0.5;
	  box_marker.color.b = 0.5;
	  box_marker.color.a = 1.0;

	  box_control.always_visible = true;
	  box_control.markers.push_back( box_marker );

	  int_marker.controls.push_back( box_control );



	  visualization_msgs::InteractiveMarkerControl control;
	  control.orientation.w = 1;
	  control.orientation.x = 1;
	  control.orientation.y = 0;
	  control.orientation.z = 0;
	  control.name = "move_x";
	  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	  int_marker.controls.push_back(control);

	  control.orientation.w = 1;
	  control.orientation.x = 0;
	  control.orientation.y = 1;
	  control.orientation.z = 0;
	  control.name = "move_y";
	  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	  int_marker.controls.push_back(control);

	  control.orientation.w = 1;
	  control.orientation.x = 0;
	  control.orientation.y = 0;
	  control.orientation.z = 1;
	  control.name = "move_z";
	  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	  int_marker.controls.push_back(control);


	  }
};


static const std::string keys[8] = {"000","001","010","011","100","101","110","111"};
struct MarkerBox {
	std::map<std::string, MoveXYZ> corners;
	interactive_markers::InteractiveMarkerServer& server;

	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
		std::string cur = feedback->marker_name;
		std::string nei;

		for (int i=0; i < 8; ++i) {
			std::string nei = keys[i];
			geometry_msgs::Pose& nei_pose = corners[nei].int_marker.pose;
			if (cur[0] == nei[0]) nei_pose.position.x = feedback->pose.position.x;
			if (cur[1] == nei[1]) nei_pose.position.y = feedback->pose.position.y;
			if (cur[2] == nei[2]) nei_pose.position.z = feedback->pose.position.z;
			server.setPose(nei, nei_pose);
		}
		server.applyChanges();
	}


	MarkerBox(interactive_markers::InteractiveMarkerServer& server_) : server(server_) {
		for (int i=0; i < 8; ++i) {
			std::string key = keys[i];
			corners[key] = MoveXYZ(key);
			if ((i/4)%2 == 1) corners[key].int_marker.pose.position.x += .1;
			if ((i/2)%2 == 1) corners[key].int_marker.pose.position.y += .1;
			if (i%2 == 1) corners[key].int_marker.pose.position.z += .1;
			server.insert(corners[key].int_marker, boost::bind(&MarkerBox::processFeedback, this, _1));
		}
	}


};






using std::string;
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
#include "utils/conversions.h"
#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
using std::runtime_error;

struct LocalConfig : Config {
  static string infile;
  static string outfile;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("i", &infile, "infile"));
    params.push_back(new Parameter<string>("o", &outfile, "outfile"));
  }
};

string LocalConfig::outfile = "";
string LocalConfig::infile = "";

Eigen::Vector3f toEigenVector(const geometry_msgs::Point& in) {
	return Eigen::Vector3f(in.x, in.y, in.z);
}

void publishCloudThread(const ColorCloudPtr& cloud) {
	ros::NodeHandle nh;

	sensor_msgs::PointCloud2 msg;
	pcl::toROSMsg(*cloud, msg);
	msg.header.frame_id = "base_link";

	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("orig_cloud", 5);
	while (ros::ok()) {
		msg.header.stamp = ros::Time::now();
		pub.publish(msg);
		sleep(1);
	}
}

int main(int argc, char** argv)
{
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);
  if (!LocalConfig::infile.size()) throw runtime_error("");
  if (!LocalConfig::outfile.size()) throw runtime_error("");

  ros::init(argc, argv, "simple_marker");


  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("simple_marker");
  MarkerBox mb(server);
  server.applyChanges();

  ColorCloudPtr cloud = readPCD(LocalConfig::infile);
  boost::thread(publishCloudThread, cloud);
  ros::spin();

  ColorCloudPtr croppedCloud = orientedBoxFilter(cloud, Eigen::Matrix3f::Identity(), toEigenVector(mb.corners["000"].int_marker.pose.position), toEigenVector(mb.corners["111"].int_marker.pose.position));
  pcl::io::savePCDFileBinary(LocalConfig::outfile, *croppedCloud);
}
