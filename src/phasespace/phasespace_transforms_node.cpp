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
#include "clouds/utils_ros.h"
#include "clouds/get_table2.h"
#include "phasespace/phasespace.h"
#include "phasespace/config_phasespace.h"

using namespace std;
using namespace Eigen;

MarkerSystemPhasespaceMsg::Ptr marker_system;

void callback(bulletsim_msgs::OWLPhasespace phasespace_msg) {
	marker_system->updateIteration(phasespace_msg);
}

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(PhasespaceConfig());
	parser.read(argc, argv);

	ros::init(argc, argv,"phasespace_transforms_node");
	ros::NodeHandle nh;

	vector<MarkerRigid::Ptr> marker_rigids;
 	for (int i=0; i<PhasespaceConfig::cameraTopics.size() && i<PhasespaceConfig::kinectInfo_filenames.size(); i++)
 		marker_rigids.push_back(createMarkerRigid(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/phasespace_rigid_info/" + PhasespaceConfig::kinectInfo_filenames[i]));

	marker_system.reset(new MarkerSystemPhasespaceMsg());
	marker_system->add(marker_rigids);
	ros::Subscriber subscriber = nh.subscribe(PhasespaceConfig::phasespaceTopic, 5, &callback);

	tf::TransformBroadcaster broadcaster;
	tf::TransformListener listener;

	ros::Rate r(30); // 30 hz
	while (ros::ok()) {
		for (int i=0; i<marker_rigids.size(); i++) {
			try {
				if (marker_rigids[i]->isValid()) {
					broadcastKinectTransform(toBulletTransform(marker_rigids[i]->getTransform()), PhasespaceConfig::cameraTopics[i]+"_rgb_optical_frame", "/ground", broadcaster, listener);
				}
			} catch (...) {
				ROS_WARN("Caught an exception from broadcastKinectTransform. Skipping...");
			}
		}

	  ros::spinOnce();
	  r.sleep();
	}
}
