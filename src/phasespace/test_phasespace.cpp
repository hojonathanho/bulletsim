#include "simulation/simplescene.h"
#include "simulation/util.h"
#include "utils/config.h"
#include "phasespace/phasespace.h"
#include "tracking/utils_tracking.h"

using namespace std;
using namespace Eigen;

void nextInd(int* ind, int increment, MarkerPointCollection::Ptr marker_points) {
	*ind = (*ind + increment + (marker_points->m_led_ids.size()+1)) % (marker_points->m_led_ids.size()+1);
	if (*ind >= 0 && *ind < marker_points->m_led_ids.size())
		printf("Current ind: %d \tLed ID: %d\n", *ind, marker_points->getLedId(*ind));
	else
		printf("Current ind: %d \tLed ID: ALL\n", *ind);
}

int main(int argc, char *argv[]) {
	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(SceneConfig());
	parser.addGroup(PhasespaceConfig());
	parser.read(argc, argv);

	Scene scene;
	scene.env->remove(scene.ground);

	vector<ledid_t> led_ids;
	for (ledid_t led_id=0; led_id<48; led_id++) led_ids.push_back(led_id);
	MarkerPointCollection::Ptr marker_points(new MarkerPointCollection(led_ids, scene.env));
	MarkerSystem::Ptr marker_system(new MarkerSystem());
	marker_system->add(marker_points);
	marker_system->startUpdateLoopThread();

	scene.startViewer();
	util::drawAxes(btTransform::getIdentity(),0.10*METERS, scene.env);
	int ind=marker_points->m_led_ids.size(); // all led_ids
	scene.addVoidKeyCallback('n',boost::bind(nextInd, &ind, +1, marker_points));
	scene.addVoidKeyCallback('b',boost::bind(nextInd, &ind, -1, marker_points));
	bool exit_loop = false;
	scene.addVoidKeyCallback('q',boost::bind(toggle, &exit_loop));

	while (!exit_loop) {
		marker_points->plot(ind);
		scene.draw();
	}
	marker_system->stopUpdateLoopThread();

	return 0;
}
