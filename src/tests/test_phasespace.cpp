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

using namespace std;
using namespace Eigen;

struct LocalConfig : Config {
  static string filename;
  static vector<ledid_t> cornersLedIds;
  static vector<ledid_t> objLedIds;
  static vector<ledid_t> kinectLedIds;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("filename", &filename, "The rigid body info is loaded from this file."));
    params.push_back(new Parameter<vector<ledid_t> >("cornersLedIds", &cornersLedIds, "Should be 4."));
    params.push_back(new Parameter<vector<ledid_t> >("objLedIds", &objLedIds, "The order matters. Even. Match front back."));
    params.push_back(new Parameter<vector<ledid_t> >("kinectLedIds", &kinectLedIds, "ID of the LEDs that are on the kinect's frame."));
  }
};

string LocalConfig::filename = "/home/alex/rll/bulletsim/data/phasespace_rigid_info/pr2head";
static const ledid_t cornersLedIds_a[4] = { 4,5,6,7 };
vector<ledid_t> LocalConfig::cornersLedIds = std::vector<ledid_t>(cornersLedIds_a, cornersLedIds_a+sizeof(cornersLedIds_a)/sizeof(ledid_t));
static const ledid_t objLedIds_a[] = { 4,5,6,7,8,9,10,11,12,13 };
vector<ledid_t> LocalConfig::objLedIds = std::vector<ledid_t>(objLedIds_a, objLedIds_a+sizeof(objLedIds_a)/sizeof(ledid_t));
static const ledid_t kinectLedIds_a[] = { 0,1,2,3 };
vector<ledid_t> LocalConfig::kinectLedIds = std::vector<ledid_t>(kinectLedIds_a, kinectLedIds_a+sizeof(kinectLedIds_a)/sizeof(ledid_t));

static const Affine3f marker_system_transform = Translation3f(-0.86564, -2.82674,  0.238075) * Quaternionf(0.603845, 0.642816, 0.319021, 0.346965);

MarkerRigid::Ptr createMarkerRigid(string rigid_info_filename, Environment::Ptr env) {
	vector<ledid_t> led_ids;
	vector<Vector3f> marker_positions;
	loadPhasespaceRigid(rigid_info_filename, led_ids, marker_positions);
	for (int i=0; i<marker_positions.size(); i++)
		marker_positions[i] *= METERS;
	MarkerRigid::Ptr marker_rigid(new MarkerRigid(led_ids, marker_positions, env));
	return marker_rigid;
}

MarkerSoft::Ptr createMarkerSoftCloth(vector<ledid_t> led_ids_front, vector<ledid_t> led_ids_back, Vector3f frontToBackVector, EnvironmentObject::Ptr sim, Environment::Ptr env) {
	assert(led_ids_front.size() == led_ids_back.size());
	vector<ledid_t> led_ids = led_ids_front;
	led_ids.insert(led_ids.end(), led_ids_back.begin(), led_ids_back.end());

	// Initialize the leds at the front
	vector<MarkerPoint::Ptr> marker_points;
	for (int ind=0; ind<led_ids_front.size(); ind++)
		marker_points.push_back(MarkerPoint::Ptr(new MarkerPoint(led_ids_front[ind], env)));
	vector<MarkerBody::Ptr> marker_bodies;
	for (int ind=0; ind<marker_points.size(); ind++)
		marker_bodies.push_back(marker_points[ind]);
	MarkerSystem::Ptr marker_system(new MarkerSystem(marker_bodies, marker_system_transform));

	// Wait until all the front leds are seen
	printf("Waiting for all the front LEDs to be seen...\n");
	marker_system->blockUntilValid();

	// For the front leds, fill marker_positions with the positions of the seen leds
	// For the back leds, assume the marker_position is frontToBackVector offset from the corresponding seen led
	vector<Vector3f> marker_positions;
	for (int ind=0; ind<led_ids_front.size(); ind++)
		marker_positions.push_back(marker_points[ind]->getPosition());
	for (int ind=0; ind<led_ids_front.size(); ind++)
		marker_positions.push_back(marker_points[ind]->getPosition() + frontToBackVector);

	MarkerSoft::Ptr marker_soft(new MarkerSoft(led_ids, marker_positions, sim, env));
	return marker_soft;
}

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

	// Phase space stuff
	vector<MarkerBody::Ptr> marker_bodies;
	vector<MarkerPoint::Ptr> marker_points;
	MarkerSystem::Ptr marker_system;

	// Initialize only the leds at the front corners
	for (int ind=0; ind<LocalConfig::cornersLedIds.size(); ind++)
		marker_points.push_back(MarkerPoint::Ptr(new MarkerPoint(LocalConfig::cornersLedIds[ind], scene.env)));
	for (int ind=0; ind<marker_points.size(); ind++)
		marker_bodies.push_back(marker_points[ind]);
	marker_system.reset(new MarkerSystem(marker_bodies, marker_system_transform));

	// Wait until the 4 leds are seen
	printf("Waiting for the 4 LEDs to be seen...\n");
	marker_system->blockUntilValid();

	// Make simulated cloth
	vector<btVector3> corners; // clockwise order
	for (int i=0; i<LocalConfig::cornersLedIds.size(); i++)
		corners.push_back(toBulletVector(marker_points[i]->getPosition() + Vector3f(0,0,-0.01*METERS)));
	BulletSoftObject::Ptr sim = makeCloth(corners, 40, 40, 1);
	scene.env->add(sim);
	sim->setColor(1,0,0,1);

	// Reset phase space stuff
	marker_system.reset();
	marker_bodies.clear();
	marker_points.clear();

	// Create rigid body for camera
	MarkerRigid::Ptr marker_rigid = createMarkerRigid(LocalConfig::filename, scene.env);
	marker_bodies.push_back(marker_rigid);

	// Create soft body for cloth
	assert(LocalConfig::objLedIds.size()%2 == 0);
	vector<ledid_t> led_ids_front(LocalConfig::objLedIds.begin(), LocalConfig::objLedIds.begin()+LocalConfig::objLedIds.size()/2.0);
	vector<ledid_t> led_ids_back(LocalConfig::objLedIds.begin()+LocalConfig::objLedIds.size()/2.0, LocalConfig::objLedIds.end());
	MarkerSoft::Ptr marker_soft = createMarkerSoftCloth(led_ids_front, led_ids_back, Vector3f(0,0,-0.02*METERS), sim, scene.env);
	marker_bodies.push_back(marker_soft);

	marker_system.reset(new MarkerSystem(marker_bodies, marker_system_transform));

	cout << "error " << marker_soft->evaluateError() << endl;

	vector<PlotAxes::Ptr> plot_axes;
	for (int ind=0; ind<LocalConfig::objLedIds.size(); ind++) {
		plot_axes.push_back(PlotAxes::Ptr (new PlotAxes()));
		scene.env->add(plot_axes[ind]);
	}

	scene.addVoidKeyCallback('q',boost::bind(exit, 0));

	scene.startViewer();

	util::drawAxes(btTransform::getIdentity(),0.10*METERS, scene.env);

	while (true) {
		//TODO the marker_system has to be in its own thread, otherwise update is very slow
		marker_system->updateMarkers();

		for (int i=0; i<marker_bodies.size(); i++)
			marker_bodies[i]->plot();
    for (int ind=0; ind<plot_axes.size(); ind++)
			plot_axes[ind]->setup(toBulletTransform(marker_soft->getSimTransform(ind)), 0.02*METERS);
    cout << "error " << marker_soft->evaluateError() << endl;

		scene.env->step(.03,2,.015);
		scene.viewer.frame();
	}

	return 0;
}
