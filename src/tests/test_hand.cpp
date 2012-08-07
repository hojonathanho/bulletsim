#include "simulation/hand.h"
#include "simulation/simplescene.h"
#include "simulation/util.h"
#include "simulation/config_bullet.h"
#include "simulation/config_viewer.h"
#include "simulation/bullet_io.h"
#include "utils/config.h"

int main(int argc, char *argv[]) {
    GeneralConfig::scale = 100.;
    BulletConfig::maxSubSteps = 0;

    Parser parser;
		parser.addGroup(GeneralConfig());
		parser.addGroup(BulletConfig());
		parser.addGroup(SceneConfig());
		parser.read(argc, argv);

		// gravity has to be reset after the parse because of the METERS scale
		BulletConfig::gravity = btVector3(0,0,-0.1*METERS); // 1/10th or /100th of actual gravity, otherwise cloth tends to go through stuff

    Scene scene;

		Hand::Ptr hand = makeHand(19, btTransform(btQuaternion::getIdentity(), btVector3(0,0,0.1*METERS)));
		scene.env->add(hand);

	  scene.addVoidKeyCallback('q',boost::bind(exit, 0));

  	scene.startViewer();
    scene.startLoop();

    return 0;
}
