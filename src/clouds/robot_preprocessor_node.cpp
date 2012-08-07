#include "utils/my_exceptions.h"
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
#include "get_table2.h"
#include <cmath>
#include "utils/config.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include "utils/conversions.h"
#include <boost/thread.hpp>
#include "table.h"

using namespace std;
using namespace Eigen;

static std::string nodeNS = "/preprocessor";
static std::string nodeName = "/preprocessor_node";

struct LocalConfig: Config {
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

	LocalConfig() :
		Config() {
		params.push_back(new Parameter<string> ("inputTopic", &inputTopic,"input topic"));
		params.push_back(new Parameter<float> ("zClipLow", &zClipLow,"clip points that are less than this much above table"));
		params.push_back(new Parameter<float> ("zClipHigh", &zClipHigh,"clip points that are more than this much above table"));
		params.push_back(new Parameter<bool> ("updateParams",&updateParams,"start a thread to periodically update the parameters thru the parameter server"));
		params.push_back(new Parameter<float> ("downsample", &downsample,"downsample voxel grid size. 0 means no"));
		params.push_back(new Parameter<bool> ("removeOutliers",&removeOutliers, "remove outliers"));
		params.push_back(new Parameter<float> ("clusterTolerance",&clusterTolerance,"points within this distance are in the same cluster"));
		params.push_back(new Parameter<float> ("clusterMinSize",&clusterMinSize,"the clusters found must have at least this number of points. 0 means no filtering"));
	}
};

string LocalConfig::inputTopic = "/camera/rgb/points";
float LocalConfig::zClipLow = .0025;
float LocalConfig::zClipHigh = 1000;
bool LocalConfig::updateParams = true;
float LocalConfig::downsample = .02;
bool LocalConfig::removeOutliers = false;
float LocalConfig::clusterTolerance = 0.03;
float LocalConfig::clusterMinSize = 40;

static int MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT, MIN_VAL, MAX_VAL;


static const float TABLE_CLUSTERING_TOLERANCE=.025; // for finding table = biggest cluster at height


template<typename T>
void getOrSetParam(const ros::NodeHandle& nh, std::string paramName, T& ref, T defaultVal) {
	if (!nh.getParam(paramName, ref)) {
		nh.setParam(paramName, defaultVal);
		ref = defaultVal;
		ROS_INFO_STREAM("setting " << paramName << " to default value " << defaultVal);
	}
}
void setParams(const ros::NodeHandle& nh) {
	getOrSetParam(nh, "min_hue", MIN_HUE, 160);
	getOrSetParam(nh, "max_hue", MAX_HUE, 10);
	getOrSetParam(nh, "min_sat", MIN_SAT, 150);
	getOrSetParam(nh, "max_sat", MAX_SAT, 255);
	getOrSetParam(nh, "min_val", MIN_VAL, 100);
	getOrSetParam(nh, "max_val", MAX_VAL, 255);
}

void setParamLoop(ros::NodeHandle& nh) {
	while (nh.ok()) {
		setParams(nh);
		sleep(1);
	}
}

class PreprocessorNode {
public:
	ros::NodeHandle& m_nh;
	ros::Publisher m_pub;
	ros::Publisher m_polyPub;
	tf::TransformListener m_listener;
	ros::Subscriber m_sub;

	bool m_inited;

	Vector3f m_mins, m_maxes;
	geometry_msgs::PolygonStamped m_polyStamped;
	geometry_msgs::Polygon m_poly;

	void callback(const sensor_msgs::PointCloud2& msg_in) {
		tf::StampedTransform st;
		try {
			m_listener.lookupTransform("base_footprint", msg_in.header.frame_id, ros::Time(0), st);
		}
		catch (tf::TransformException e) {
			printf("tf error: %s\n", e.what());
			return;
		}
		Affine3f baseFootprintFromCam = toEigenTransform(st.asBt());

		ColorCloudPtr cloud_in(new ColorCloud());
		pcl::fromROSMsg(msg_in, *cloud_in);
		cloud_in = transformPointCloud1(cloud_in, baseFootprintFromCam);

		if (!m_inited) {
			initTable(cloud_in);
		}

		ColorCloudPtr cloud_out = boxFilter(cloud_in, m_mins, m_maxes);
		cloud_out = hueFilter(cloud_out, MIN_HUE, MAX_HUE, MIN_SAT, MAX_SAT,MIN_VAL, MAX_VAL);
		if (LocalConfig::downsample > 0) cloud_out = downsampleCloud(cloud_out, LocalConfig::downsample);
		if (LocalConfig::removeOutliers) cloud_out = removeOutliers(cloud_out, 1, 10);
		if (LocalConfig::clusterMinSize > 0 && cloud_out->size() > 50) {
			cloud_out = clusterFilter(cloud_out, LocalConfig::clusterTolerance,LocalConfig::clusterMinSize);
		}
		sensor_msgs::PointCloud2 msg_out;
		pcl::toROSMsg(*cloud_out, msg_out);
		msg_out.header.frame_id = "base_footprint";
		m_pub.publish(msg_out);

		m_polyPub.publish(m_polyStamped);

	}

	void initTable(ColorCloudPtr transformed_cloud) {
		float z_table = getTableHeight(transformed_cloud);
		transformed_cloud = downsampleCloud(transformed_cloud, .01);
		ColorCloudPtr in_table = getTablePoints(transformed_cloud, z_table);
		in_table = getBiggestCluster(in_table, TABLE_CLUSTERING_TOLERANCE);

		float xmin, xmax, ymin, ymax;
		getTableXYBounds(in_table, xmin, xmax, ymin, ymax);
		m_mins = Vector3f(xmin, ymin, z_table+LocalConfig::zClipLow);
		m_maxes = Vector3f(xmax, ymax, z_table+LocalConfig::zClipHigh);

		vector<geometry_msgs::Point32>& poly = m_polyStamped.polygon.points;
		geometry_msgs::Point32 p;
		p.z = z_table;
		p.x = xmin;
		p.y = ymin;
		poly.push_back(p);
		p.x = xmax;
		p.y = ymin;
		poly.push_back(p);
		p.x = xmax;
		p.y = ymax;
		poly.push_back(p);
		p.x = xmin;
		p.y = ymax;
		poly.push_back(p);

		m_inited = true;

	}

	PreprocessorNode(ros::NodeHandle& nh) :
		m_inited(false), m_nh(nh), m_pub(
				nh.advertise<sensor_msgs::PointCloud2> ("points", 5)),
				m_polyPub(nh.advertise<geometry_msgs::PolygonStamped> ("polygon", 5)), m_sub(nh.subscribe(LocalConfig::inputTopic, 1,&PreprocessorNode::callback, this))
	{
		m_polyStamped.header.frame_id = "base_footprint";
	}

};

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	ros::init(argc, argv, "preprocessor");
	ros::NodeHandle nh(nodeNS);

	setParams(nh);
	if (LocalConfig::updateParams)
		boost::thread setParamThread(setParamLoop, nh);

	PreprocessorNode tp(nh);
	ros::spin();
}
