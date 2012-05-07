#include <clouds/utils_pcl.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <cloud_defs/GetTable.h>
#include <pcl/ros/conversions.h>
#include "clouds/get_table2.h"
#include "geometry_msgs/Point.h"
#include <pcl/visualization/cloud_viewer.h>
#include <utils/config.h>

using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
  static bool view;
  LocalConfig() : Config() {
    params.push_back(new Parameter<bool>("view",&view,"show viewer"));
  }
};
bool LocalConfig::view = false;

struct GetTableServer {
public:
	ros::NodeHandle& m_nh;
	ros::Publisher m_pub;
	GetTableServer(ros::NodeHandle& nh) :
		m_nh(nh),
		m_pub(nh.advertise<geometry_msgs::PolygonStamped>("table",1))
	{}
	bool callback(cloud_defs::GetTable::Request& request, cloud_defs::GetTable::Response& response) {
	  std::string topicName("/camera/depth_registered/points");
	  sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topicName, m_nh, ros::Duration(1));
	  if (msg) {
		  ColorCloudPtr cloud(new ColorCloud());
		  pcl::fromROSMsg(*msg, *cloud);
		  Matrix3f corners = getTableCornersRansac(cloud);

		  response.poly.polygon.points.resize(corners.size());
		  for (int i=0; i < corners.size(); i++) {
			  geometry_msgs::Point32& p = response.poly.polygon.points[i];
			  p.x = corners(i,0);
			  p.y = corners(i,1);
			  p.z = corners(i,2);
		  }
		  response.poly.header = msg->header;
		  m_pub.publish(response.poly);
		  return true;
	  }
	  else {
		  ROS_ERROR("GetTableServer didn't receive any point clouds");
		  return false;
	  }

	}
};

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "get_table_server");
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	ros::NodeHandle nh;
	GetTableServer server(nh);
	ros::ServiceServer service = nh.advertiseService("get_table", &GetTableServer::callback, &server);
	ros::spin();

}
