#include "simulation/simplescene.h"
#include "simulation/util.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "simulation/bullet_io.h"
#include "utils/config.h"
#include "phasespace/phasespace.h"
#include "utils/conversions.h"
#include "simulation/plotting.h"
#include "simulation/softbodies.h"
#include "tracking/tracked_object.h"
#include "tracking/utils_tracking.h"
#include "tracking/config_tracking.h"

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
	GeneralConfig::scale = 100;
	BulletConfig::maxSubSteps = 0;

	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(SceneConfig());
	parser.addGroup(PhasespaceConfig());
	parser.read(argc, argv);

	Scene scene;
	scene.env->remove(scene.ground);

	MarkerSystem::Ptr marker_system(new MarkerSystem());
	marker_system->add(createMarkerRigid(PhasespaceConfig::kinectInfo_filenames[0], scene.env));
	marker_system->startUpdateLoopThread();

	scene.startViewer();
	util::drawAxes(btTransform::getIdentity(),0.10*METERS, scene.env);
	bool exit_loop = false;
	scene.addVoidKeyCallback('q',boost::bind(toggle, &exit_loop));

	while (!exit_loop) {
		marker_system->plot();

		scene.draw();
	}
	marker_system->stopUpdateLoopThread();

	return 0;
}
