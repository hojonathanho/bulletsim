#include "simulation/hand.h"
#include "simulation/simplescene.h"
#include "simulation/util.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "simulation/bullet_io.h"
#include "utils/config.h"

struct LocalConfig: Config {
	static float erp;
	static float cmp;
	LocalConfig() : Config() {
    params.push_back(new Parameter<float>("erp", &erp, "parameter for the hand hinge constraints"));
    params.push_back(new Parameter<float>("cmp", &cmp, "parameter for the hand hinge constraints"));
  }
};
float LocalConfig::erp = 0.1;
float LocalConfig::cmp = 0.8;

int main(int argc, char *argv[]) {
  GeneralConfig::scale = 10;

	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(SceneConfig());
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	Scene scene;


	btTransform initHandTrans = btTransform(btQuaternion(btVector3(1,0,0), -M_PI/2.0), btVector3(0,0,0.2*METERS));
	HumanHandObject::Ptr hand(new HumanHandObject(scene.rave, initHandTrans, LocalConfig::erp, LocalConfig::cmp));
	scene.env->add(hand);
	cout << "joint angles " << hand->getJointAngles() << endl;

	scene.addVoidKeyCallback('q',boost::bind(exit, 0));

	scene.startViewer();
	scene.startLoop();

	return 0;
}
