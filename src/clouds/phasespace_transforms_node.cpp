#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "clouds/utils_pcl.h"
#include "utils/config.h"
#include "utils/conversions.h"
#include "utils_ros.h"
#include "clouds/get_table2.h"
#include "tracking/phasespace.h"

using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
  static vector<string> cameraTopics;
  static vector<string> rigidInfos;

  LocalConfig() : Config() {
    params.push_back(new Parameter<vector<string> >("cameraTopics", &cameraTopics, "camera base topics. there should be at least two."));
    params.push_back(new Parameter<vector<string> >("rigidInfos", &rigidInfos, "camera rigid body info filename."));
  }
};

//string cameraTopics_a[] = { "/kinect1/depth_registered/points", "/kinect2/depth_registered/points" };
string cameraTopics_a[] = { "/kinect1/depth_registered/points" };
vector<string> LocalConfig::cameraTopics = vector<string>(cameraTopics_a, cameraTopics_a+sizeof(cameraTopics_a)/sizeof(string));
//string rigidInfos_a[] = { "/home/alex/rll/bulletsim/data/phasespace_rigid_info/pr2head", "/home/alex/rll/bulletsim/data/phasespace_rigid_info/tripod" };
string rigidInfos_a[] = { "/home/alex/rll/bulletsim/data/phasespace_rigid_info/pr2head" };
vector<string> LocalConfig::rigidInfos = vector<string>(rigidInfos_a, rigidInfos_a+sizeof(rigidInfos_a)/sizeof(string));

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	ros::init(argc, argv,"phasespace_transforms_node");
	ros::NodeHandle nh;

	assert(LocalConfig::cameraTopics.size() == LocalConfig::rigidInfos.size());

	vector<sensor_msgs::PointCloud2ConstPtr> cloud_msgs;

	vector<MarkerRigid::Ptr> marker_rigids;
	vector<MarkerBody::Ptr> marker_bodies;

 	for (int i=0; i<LocalConfig::cameraTopics.size(); i++) {
 		cloud_msgs.push_back(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(LocalConfig::cameraTopics[i], nh));
 		MarkerRigid::Ptr marker_rigid = createMarkerRigid(LocalConfig::rigidInfos[i]);
 		marker_rigids.push_back(marker_rigid);
 		marker_bodies.push_back(marker_rigid);
 	}
 	MarkerSystem::Ptr marker_system(new MarkerSystem(marker_bodies));

	tf::TransformBroadcaster broadcaster;
	tf::TransformListener listener;

	ros::Rate r(1000); // 10 hz
	while (ros::ok()) {
		marker_system->updateMarkers();
		for (int i=0; i<marker_rigids.size(); i++) {
			try {
				if (marker_rigids[i]->isValid()) {
					broadcastKinectTransform(toBulletTransform(marker_rigids[i]->getTransform()), cloud_msgs[i]->header.frame_id, "/ground", broadcaster, listener);
				}
			} catch (...) {
				ROS_WARN("Caught an exception from broadcastKinectTransform. Skipping...");
			}
		}

	  ros::spinOnce();
	  r.sleep();
	}
}
