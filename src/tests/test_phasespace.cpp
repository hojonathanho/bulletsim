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
static const ledid_t cornersLedIds_a[4] = { 4,5,6,7 };
vector<ledid_t> LocalConfig::cornersLedIds = std::vector<ledid_t>(cornersLedIds_a, cornersLedIds_a+sizeof(cornersLedIds_a)/sizeof(ledid_t));
static const ledid_t objLedIds_a[] = { 4,5,6,7,8,9,10,11,12,13 };
vector<ledid_t> LocalConfig::objLedIds = std::vector<ledid_t>(objLedIds_a, objLedIds_a+sizeof(objLedIds_a)/sizeof(ledid_t));

static const Affine3f marker_system_transform = Translation3f(-0.86564, -2.82674,  0.238075) * Quaternionf(0.603845, 0.642816, 0.319021, 0.346965);

vector<Vector3f> waitForMarkerPositions(vector<ledid_t> led_ids) {
	vector<MarkerPoint::Ptr> marker_points;
	for (int ind=0; ind<led_ids.size(); ind++)
		marker_points.push_back(MarkerPoint::Ptr(new MarkerPoint(led_ids[ind])));
	vector<MarkerBody::Ptr> marker_bodies;
	for (int ind=0; ind<marker_points.size(); ind++)
		marker_bodies.push_back(marker_points[ind]);

	MarkerSystem::Ptr marker_system(new MarkerSystem(marker_bodies, marker_system_transform));

	cout << "Waiting for LEDs " << led_ids << " to be seen..." << endl;
	marker_system->blockUntilAllValid();

	vector<Vector3f> marker_positions;
	for (int ind=0; ind<led_ids.size(); ind++)
		marker_positions.push_back(marker_points[ind]->getPosition());
	return marker_positions;
}

MarkerRigid::Ptr createMarkerRigid(string rigid_info_filename, Environment::Ptr env=Environment::Ptr()) {
	vector<ledid_t> led_ids;
	vector<Vector3f> marker_positions;
	loadPhasespaceRigid(rigid_info_filename, led_ids, marker_positions);
	for (int i=0; i<marker_positions.size(); i++)
		marker_positions[i] *= METERS;
	MarkerRigid::Ptr marker_rigid(new MarkerRigid(led_ids, marker_positions, env));
	return marker_rigid;
}

MarkerSoft::Ptr createMarkerSoftCloth(vector<ledid_t> led_ids_front, vector<ledid_t> led_ids_back, Vector3f frontToBackVector, EnvironmentObject::Ptr sim, Environment::Ptr env=Environment::Ptr()) {
	assert(led_ids_front.size() == led_ids_back.size());

	// For the front LEDs, fill marker_positions with the positions of the seen LEDs
	// For the back LEDs, assume the marker_position is frontToBackVector offset from the corresponding seen LED
	vector<Vector3f> marker_positions = waitForMarkerPositions(led_ids_front);
	for (int ind=0; ind<led_ids_front.size(); ind++)
		marker_positions.push_back(marker_positions[ind] + frontToBackVector);

	// Append front and back LEDs
	vector<ledid_t> led_ids = led_ids_front;
	led_ids.insert(led_ids.end(), led_ids_back.begin(), led_ids_back.end());

	MarkerSoft::Ptr marker_soft(new MarkerSoft(led_ids, marker_positions, sim, env));
	return marker_soft;
}

bool exit_loop = false;

void startMarkerUpdateLoop(MarkerSystem::Ptr marker_system) {
	while(true) {
		if (exit_loop) break;

		marker_system->updateMarkers();
		usleep(1000);
	}
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

	// Get the positions of the LEDs at the front corners only and make simulated cloth
	cout << "Finding cloth corners to create cloth..." << endl;
	vector<Vector3f> marker_positions = waitForMarkerPositions(LocalConfig::cornersLedIds);
	vector<btVector3> corners; // clockwise order
	BOOST_FOREACH(Vector3f position, marker_positions)
		corners.push_back(toBulletVector(position + Vector3f(0,0,-0.01*METERS)));
	BulletSoftObject::Ptr sim = makeCloth(corners, 40, 40, 1);
	scene.env->add(sim);
	sim->setColor(1,0,0,1);

	// Contains all the phase space bodies
	vector<MarkerBody::Ptr> marker_bodies;

	// Create rigid body for camera
	MarkerRigid::Ptr marker_rigid = createMarkerRigid(LocalConfig::kinectInfo_filename, scene.env);
	marker_bodies.push_back(marker_rigid);

	// Create soft body for cloth
	cout << "Creating soft body cloth..." << endl;
	assert(LocalConfig::objLedIds.size()%2 == 0);
	vector<ledid_t> led_ids_front(LocalConfig::objLedIds.begin(), LocalConfig::objLedIds.begin()+LocalConfig::objLedIds.size()/2.0);
	vector<ledid_t> led_ids_back(LocalConfig::objLedIds.begin()+LocalConfig::objLedIds.size()/2.0, LocalConfig::objLedIds.end());
	MarkerSoft::Ptr marker_soft = createMarkerSoftCloth(led_ids_front, led_ids_back, Vector3f(0,0,-0.02*METERS), sim, scene.env);
	marker_bodies.push_back(marker_soft);

	MarkerSystem::Ptr marker_system(new MarkerSystem(marker_bodies, marker_system_transform));

	cout << "error " << marker_soft->evaluateError() << endl;

	vector<PlotAxes::Ptr> plot_axes;
	for (int ind=0; ind<LocalConfig::objLedIds.size(); ind++) {
		plot_axes.push_back(PlotAxes::Ptr (new PlotAxes()));
		scene.env->add(plot_axes[ind]);
	}

	scene.startViewer();

	util::drawAxes(btTransform::getIdentity(),0.10*METERS, scene.env);

  boost::shared_ptr<boost::thread> markerThread = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&startMarkerUpdateLoop, marker_system)));

	scene.addVoidKeyCallback('q',boost::bind(toggle, &exit_loop));

	while (true) {
		if (exit_loop) break;
		for (int i=0; i<marker_bodies.size(); i++)
			marker_bodies[i]->plot();
    for (int ind=0; ind<plot_axes.size(); ind++)
			plot_axes[ind]->setup(toBulletTransform(marker_soft->getSimTransform(ind)), 0.02*METERS);
    cout << "error " << marker_soft->evaluateError() << endl;

		scene.env->step(.03,2,.015);
		scene.viewer.frame();
	}
	markerThread->join();

	return 0;
}
