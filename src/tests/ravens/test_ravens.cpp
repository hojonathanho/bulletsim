#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/softBodyHelpers.h"
#include "simulation/basicobjects.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <openrave/kinbody.h>
#include "robots/pr2.h"
#include "CustomScene.h"
#include "ravens_config.h"


int main(int argc, char *argv[]) {
	ViewerConfig::cameraHomePosition = btVector3(0, 0.6, 0.5);
	ViewerConfig::cameraHomeCenter = btVector3(0, 0, 0.25);
	BulletConfig::dt = 0.01;

	BulletConfig::internalTimeStep = 0.001;
	BulletConfig::maxSubSteps = 10;
	BulletConfig::gravity = -1*btVector3(0,0,10);
	RavenConfig::record_freq = 100;
	RavenConfig::cloth = 0;
	SceneConfig::enableHaptics = 1;
	SceneConfig::enableShadows = 1;
	GeneralConfig::scale = 100.;

	Parser parser;

	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(SceneConfig());
	parser.addGroup(RavenConfig());

	parser.read(argc, argv);

	CustomScene().run();
	return 0;
}

