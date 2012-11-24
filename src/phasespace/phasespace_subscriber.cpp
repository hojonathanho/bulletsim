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

MarkerSystemPhasespaceMsg::Ptr marker_system;

void callback(bulletsim_msgs::OWLPhasespace phasespace_msg) {
	marker_system->updateIteration(phasespace_msg);
}

int main(int argc, char *argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.addGroup(PhasespaceConfig());
	parser.read(argc, argv);

	ros::init(argc, argv,"phasespace_subscriber");
	ros::NodeHandle nh;

	Scene scene;
	scene.env->remove(scene.ground);

	vector<ledid_t> led_ids;
	for (ledid_t led_id=0; led_id<48; led_id++) led_ids.push_back(led_id);
	MarkerPointCollection::Ptr marker_points(new MarkerPointCollection(led_ids, scene.env));

	marker_system.reset(new MarkerSystemPhasespaceMsg());
	marker_system->add(marker_points);
	for (int i=0; i<PhasespaceConfig::kinectInfo_filenames.size(); i++)
		marker_system->add(createMarkerRigid(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/phasespace_rigid_info/" + PhasespaceConfig::kinectInfo_filenames[i], scene.env));

	ros::Subscriber subscriber = nh.subscribe(PhasespaceConfig::phasespaceTopic, 5, &callback);

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

	return 0;
}
