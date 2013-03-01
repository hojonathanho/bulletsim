#include "simulation/simplescene.h"
#include "BoxCloth.h"
#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
#include "simulation/config_bullet.h"
#include "simulation/softBodyHelpers.h"
#include "simulation/basicobjects.h"
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <openrave/kinbody.h>
#include "robots/pr2.h"
#include "CustomScene.h"

struct LocalConfig : Config {
	static int n;
	static int m;
	static float s;
	static float h;

	LocalConfig() : Config() {
		params.push_back(new Parameter<int>("n", &n, "number of squares in x direction"));
	    params.push_back(new Parameter<int>("m", &m, "number of squares in y direction"));
	    params.push_back(new Parameter<float>("s", &s, "square side length"));
	    params.push_back(new Parameter<float>("h", &h, "box object height"));
	  }
};

int LocalConfig::n = 5;
int LocalConfig::m = 5;
float LocalConfig::s = 0.05;
float LocalConfig::h = 0.001;

int main(int argc, char* argv[]) {
	GeneralConfig::scale = 20.;
	ViewerConfig::cameraHomePosition = btVector3(100, 0, 100);
	BulletConfig::dt = 0.01;
	BulletConfig::internalTimeStep = 0.001;
	BulletConfig::maxSubSteps = 10;

	Parser parser;

	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(SceneConfig());
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);


	Scene scene;

	vector<unsigned int> hole_x;
	hole_x.push_back(1);
	hole_x.push_back(2);
	vector<unsigned int> hole_y;
	hole_y.push_back(2);
	hole_y.push_back(5);

	BoxCloth::Ptr cloth(new BoxCloth(LocalConfig::n,LocalConfig::m, hole_x, hole_y, LocalConfig::s,LocalConfig::h));

	scene.env->add(cloth);
	scene.startViewer();
	scene.startLoop();
}
