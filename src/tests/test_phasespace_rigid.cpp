#include "simulation/simplescene.h"
#include "simulation/util.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "simulation/bullet_io.h"
#include "utils/config.h"
#include "tracking/phasespace.h"
#include "utils/conversions.h"
#include "simulation/plotting.h"
#include "simulation/softbodies.h"
#include "tracking/tracked_object.h"
#include "tracking/utils_tracking.h"

using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
  static string kinectInfo_filename;
  static vector<ledid_t> cornersLedIds;
  static vector<ledid_t> objLedIds;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("kinectInfo_filename", &kinectInfo_filename, "The rigid body info is loaded from this file."));
    params.push_back(new Parameter<vector<ledid_t> >("cornersLedIds", &cornersLedIds, "Should be 4."));
    params.push_back(new Parameter<vector<ledid_t> >("objLedIds", &objLedIds, "The order matters. Even. Match front back."));
  }
};

string LocalConfig::kinectInfo_filename = "/home/alex/rll/bulletsim/data/phasespace_rigid_info/pr2head";
static const ledid_t cornersLedIds_a[4] = { 8,9,10,11 };
vector<ledid_t> LocalConfig::cornersLedIds = std::vector<ledid_t>(cornersLedIds_a, cornersLedIds_a+sizeof(cornersLedIds_a)/sizeof(ledid_t));
static const ledid_t objLedIds_a[] = { 8,9,10,11,12, 13,14,15,16,17 };
vector<ledid_t> LocalConfig::objLedIds = std::vector<ledid_t>(objLedIds_a, objLedIds_a+sizeof(objLedIds_a)/sizeof(ledid_t));

static const Affine3f marker_system_transform = Translation3f(-0.86564, -2.82674,  0.238075) * Quaternionf(0.603845, 0.642816, 0.319021, 0.346965);

int main(int argc, char *argv[]) {
	GeneralConfig::scale = 100;
	BulletConfig::maxSubSteps = 0;

	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(SceneConfig());
  parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	Scene scene;

	// Contains all the phase space bodies
	vector<MarkerBody::Ptr> marker_bodies;

	// Create rigid body for camera
	MarkerRigid::Ptr marker_rigid = createMarkerRigid(LocalConfig::kinectInfo_filename, scene.env);
	marker_bodies.push_back(marker_rigid);

	MarkerSystem::Ptr marker_system(new MarkerSystem(marker_bodies));
	marker_system->setPose(marker_system_transform);

	vector<PlotAxes::Ptr> plot_axes;
	for (int ind=0; ind<LocalConfig::objLedIds.size(); ind++) {
		plot_axes.push_back(PlotAxes::Ptr (new PlotAxes()));
		scene.env->add(plot_axes[ind]);
	}

	scene.startViewer();

	util::drawAxes(btTransform::getIdentity(),0.10*METERS, scene.env);

	marker_system->startUpdateLoopThread();

	bool exit_loop = false;
	scene.addVoidKeyCallback('q',boost::bind(toggle, &exit_loop));

	while (!exit_loop) {
		for (int i=0; i<marker_bodies.size(); i++)
			marker_bodies[i]->plot();

		scene.env->step(.03,2,.015);
		scene.viewer.frame();
	}
	marker_system->stopUpdateLoopThread();

	return 0;
}
