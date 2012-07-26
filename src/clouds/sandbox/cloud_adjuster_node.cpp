#include "utils/my_exceptions.h"
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
#include "utils/config.h"


using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
  static std::string inputTopic;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("inputTopic", &inputTopic, "input topic"));
  }
};
string LocalConfig::inputTopic = "/camera/depth_registered/points";

class PreprocessorNode {
public:
	ros::NodeHandle& m_nh;
	ros::Publisher m_pub;
	ros::Subscriber m_sub;

	MatrixXf m_mat;

	void callback(const sensor_msgs::PointCloud2& msg_in) {

		ColorCloudPtr cloud_in(new ColorCloud());
		pcl::fromROSMsg(msg_in, *cloud_in);

		BOOST_FOREACH(ColorPoint& pt, cloud_in->points) {
			Vector3f v = m_mat * pt.getVector4fMap();
			pt.x = v(0);
			pt.y = v(1);
			pt.z = v(2);
		}



	    sensor_msgs::PointCloud2 msg_out;
	    pcl::toROSMsg(*cloud_in, msg_out);
	    msg_out.header = msg_in.header;
	    m_pub.publish(msg_out);
	}


	PreprocessorNode(ros::NodeHandle& nh) :
		m_nh(nh),
		m_pub(nh.advertise<sensor_msgs::PointCloud2> ("points", 5)),
		m_sub(nh.subscribe(LocalConfig::inputTopic, 1, &PreprocessorNode::callback, this)),
		m_mat(3,4)
	{
//		m_mat <<  0.977,  0.006,  0.044, -0.039,
//				  -0.003,  0.988, -0.013,  0.016,
//			       -0.032, -0.038,  1.015, -0.011;
		m_mat << 0.977,  0.009,  0.037, -0.032,
				-0.008,  0.991, -0.017,  0.012,
			       -0.027, -0.028,  1.017, -0.013;
	}

};

int main(int argc, char* argv[]) {

  Parser parser;
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);


	ros::init(argc, argv, "preprocessor");
	ros::NodeHandle nh("cloud_adjuster");

	PreprocessorNode preproc(nh);
	ros::spin();
}
