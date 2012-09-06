#include "simulation/simplescene.h"
#include "utils/config.h"
#include "tracking/phasespace.h"
#include "tracking/utils_tracking.h"

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(SceneConfig());
	parser.read(argc, argv);

	Scene scene;
	scene.env->remove(scene.ground);

	vector<ledid_t> led_ids;
	for (ledid_t led_id=0; led_id<72; led_id++) led_ids.push_back(led_id);
	MarkerPointCollection::Ptr marker_points(new MarkerPointCollection(led_ids, scene.env));
	MarkerSystem::Ptr marker_system(new MarkerSystem(vector<MarkerBody::Ptr>(1, marker_points)));
	marker_system->startUpdateLoopThread();

	scene.startViewer();
	util::drawAxes(btTransform::getIdentity(),0.10*METERS, scene.env);
	bool exit_loop = false;
	scene.addVoidKeyCallback('q',boost::bind(toggle, &exit_loop));

	while (!exit_loop) {
		marker_points->plot();
		scene.draw();
	}
  marker_system->stopUpdateLoopThread();

	return 0;
}
