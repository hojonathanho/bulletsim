#include "utils/my_exceptions.h"
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
#include "get_table2.h"
#include <cmath>

using namespace std;
using namespace Eigen;

class TowelPreprocessorROS {
public:
	ros::NodeHandle& m_nh;
	ros::Publisher m_pub;
	ros::Subscriber m_sub;

	bool m_inited;

	Matrix3f m_axes;
	Vector3f m_mins, m_maxes;


	void callback(const sensor_msgs::PointCloud2& msg_in) {
		ColorCloudPtr cloud_in(new ColorCloud());
		pcl::fromROSMsg(msg_in, *cloud_in);

		if (!m_inited) {
			initTable(cloud_in);
		}

		ColorCloudPtr cloud_out = orientedBoxFilter(cloud_in, m_axes, m_mins, m_maxes);
		cloud_out = hueFilter(cloud_out, 170, 10);



		sensor_msgs::PointCloud2 msg_out;
		pcl::toROSMsg(*cloud_out, msg_out);
		m_pub.publish((msg_out));
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
		m_mins(2) += .0025;
		m_maxes(2) = 10000;

		m_inited = true;

	}

	TowelPreprocessorROS(ros::NodeHandle& nh) :
		m_inited(false),
		m_nh(nh),
		m_pub(nh.advertise<sensor_msgs::PointCloud2>("cloud_out",5)),
		m_sub(nh.subscribe("cloud_in", 1, &TowelPreprocessorROS::callback, this))
	{
	}


};

int main(int argc, char* argv[]) {
  // po::options_description opts("Allowed options");
  // opts.add_options()
  //   ("help,h", "produce help message")
  // po::variables_map vm;        
  // po::store(po::command_line_parser(argc, argv)
  // 	    .options(opts)
  // 	    .run()
  // 	    , vm);
  // if (vm.count("help")) {
  //   cout << "usage: comm_downsample_clouds [options]" << endl;
  //   cout << opts << endl;
  //   return 0;
  // }
  // po::notify(vm);
 


  ros::init(argc, argv,"towel_preprocessor");
  ros::NodeHandle nh;

  TowelPreprocessorROS tp(nh);
  ros::spin();
}
