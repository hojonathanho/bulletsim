#include <iostream>
#include <ros/ros.h>
#include "simulation/simplescene.h"
#include "simulation/util.h"
#include "utils/config.h"
#include "tracking/utils_tracking.h"
#include "phasespace/phasespace.h"
#include "tracking/config_tracking.h"

using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
  static bool visualize;

  LocalConfig() : Config() {
    params.push_back(new Parameter<bool>("visualize", &visualize, "If true, the points are visualized in an scene."));
  }
};

bool LocalConfig::visualize = true;

int main(int argc, char *argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.addGroup(GeneralConfig());
	parser.addGroup(SceneConfig());
	parser.addGroup(PhasespaceConfig());
	parser.read(argc, argv);

	ros::init(argc, argv,"phasespace_publisher");
	ros::NodeHandle nh;

	Scene scene;
	scene.env->remove(scene.ground);

	vector<ledid_t> led_ids;
	for (ledid_t led_id=0; led_id<48; led_id++) led_ids.push_back(led_id);
	MarkerPointCollection::Ptr marker_points(new MarkerPointCollection(led_ids, scene.env));
	boost::shared_ptr<ros::Publisher> publisher(new ros::Publisher(nh.advertise<bulletsim_msgs::OWLPhasespace>( PhasespaceConfig::phasespaceTopic, 5 )));
	MarkerSystemPublisher::Ptr marker_system(new MarkerSystemPublisher(publisher));
	marker_system->add(marker_points);
	marker_system->startUpdateLoopThread();

	if (LocalConfig::visualize) scene.startViewer();
	util::drawAxes(btTransform::getIdentity(),0.10*METERS, scene.env);
	bool exit_loop = false;
	scene.addVoidKeyCallback('q',boost::bind(toggle, &exit_loop));

	ros::Rate rate(30);
	while (ros::ok() && !exit_loop) {
		if (LocalConfig::visualize) {
			marker_system->plot();
			scene.draw();
		}

		ros::spinOnce();
		rate.sleep();
	}
	marker_system->stopUpdateLoopThread();

	return 0;
}
