#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/common/transforms.h>
#include "clouds/utils_ros.h"
#include "simulation/simplescene.h"
#include "simulation/util.h"
#include "utils/config.h"
#include "phasespace/phasespace.h"
#include "simulation/plotting.h"
#include "tracking/plotting_tracking.h"
#include <boost/assign/list_of.hpp>

using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
	static std::vector<std::string> cloudTopics;
	static string groundFrame;

  LocalConfig() : Config() {
    params.push_back(new ParameterVec<std::string>("cloudTopics", &cloudTopics, "camera full topics"));
    params.push_back(new Parameter<string> ("groundFrame", &groundFrame, "the polygon points are saved with respect to this frame"));
  }
};

std::vector<std::string> LocalConfig::cloudTopics = boost::assign::list_of("/drop/kinect1/points")("/drop/kinect2/points");
string LocalConfig::groundFrame = "/ground";

vector<btTransform> groundFromCameras;
ColorCloudPtr cloud(new ColorCloud());
MarkerSystemPhasespaceMsg::Ptr marker_system;

void cloudPhasespaceCallback(const sensor_msgs::PointCloud2ConstPtr& cloud0_msg, const sensor_msgs::PointCloud2ConstPtr& cloud1_msg, const bulletsim_msgs::OWLPhasespaceConstPtr& phasespace_msg) {
	ColorCloudPtr local_cloud0(new ColorCloud());
	pcl::fromROSMsg(*cloud0_msg, *local_cloud0);
	pcl::transformPointCloud(*local_cloud0, *local_cloud0, toEigenTransform(groundFromCameras[0]));

	ColorCloudPtr local_cloud1(new ColorCloud());
	pcl::fromROSMsg(*cloud1_msg, *local_cloud1);
	pcl::transformPointCloud(*local_cloud1, *local_cloud1, toEigenTransform(groundFromCameras[1]));

	*cloud = *local_cloud0 + *local_cloud1;

	marker_system->updateIteration(*phasespace_msg);
}

int main(int argc, char *argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.addGroup(PhasespaceConfig());
	parser.read(argc, argv);

	ros::init(argc, argv,"phasespace_cloud_viz");
	ros::NodeHandle nh;

	assert(LocalConfig::cloudTopics.size() == 2);

	tf::TransformListener listener;
	for (int i=0; i<LocalConfig::cloudTopics.size(); i++)
		groundFromCameras.push_back(waitForAndGetTransform(listener, LocalConfig::groundFrame, ros::topic::waitForMessage<sensor_msgs::PointCloud2>(LocalConfig::cloudTopics[i], nh)->header.frame_id));

	Scene scene;

	vector<ledid_t> led_ids;
	for (ledid_t led_id=0; led_id<48; led_id++) led_ids.push_back(led_id);
	MarkerPointCollection::Ptr marker_points(new MarkerPointCollection(led_ids, scene.env));

	marker_system.reset(new MarkerSystemPhasespaceMsg());
	marker_system->add(marker_points);
	for (int i=0; i<PhasespaceConfig::kinectInfo_filenames.size(); i++)
		marker_system->add(createMarkerRigid(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/phasespace_rigid_info/" + PhasespaceConfig::kinectInfo_filenames[i], scene.env));

	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub0(nh, LocalConfig::cloudTopics[0], 20);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub1(nh, LocalConfig::cloudTopics[1], 20);
	message_filters::Subscriber<bulletsim_msgs::OWLPhasespace> phasespaceSub(nh, PhasespaceConfig::phasespaceTopic, 20);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2, bulletsim_msgs::OWLPhasespace> SyncPolicy;
	message_filters::Synchronizer<SyncPolicy>* sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(30), cloudSub0, cloudSub1, phasespaceSub);
	sync->registerCallback(boost::bind(&cloudPhasespaceCallback,_1,_2,_3));

	scene.startViewer();
	util::drawAxes(btTransform::getIdentity(),0.10*METERS, scene.env);
	PointCloudPlot::Ptr cloud_plot(new PointCloudPlot(3));
	scene.env->add(cloud_plot);

	bool exit_loop = false;
	scene.addVoidKeyCallback('q',boost::bind(toggle, &exit_loop));

//	ros::Rate rate(30);
	while (ros::ok() && !exit_loop) {
		marker_system->plot();
		cloud_plot->setPoints1(cloud,0.5);
		scene.draw();

		ros::spinOnce();
		//rate.sleep();
	}

	return 0;
}
