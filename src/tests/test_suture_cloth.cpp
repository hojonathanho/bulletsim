#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/softBodyHelpers.h"
#include "simulation/basicobjects.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <openrave/kinbody.h>
#include "robots/ravens.h"
#include "CustomScene.h"



int main(int argc, char *argv[]) {
	GeneralConfig::scale = 20.;
	ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
	BulletConfig::dt = 0.01;
	BulletConfig::internalTimeStep = 0.01;
	BulletConfig::maxSubSteps = 0;

	Parser parser;

	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(SceneConfig());
	parser.read(argc, argv);

	CustomScene().run();
	return 0;
}

