/*
 * test_multihyp_tracker.cpp
 *
 *  Created on: Nov 7, 2012
 *      Author: alex
 */

#include "simulation/simplescene.h"
#include "robots/pr2.h"

int main(int argc, char *argv[]) {
    // first read the configuration from the user

    // and override config values to what we want
    SceneConfig::enableIK = true;
    SceneConfig::enableRobot = true;
    SceneConfig::enableHaptics = true;

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);

    // construct the scene
    Scene scene;

    // manipulate the scene or add more objects, if desired
    PR2Manager pr2m(scene);
//    Load(scene.env, scene.rave, EXPAND(BULLETSIM_DATA_DIR)"/xml/table.xml");
//    RaveObject::Ptr table = getObjectByName(scene.env, scene.rave, "table");
//    table->setColor(0,.8,0,.5);
    pr2m.pr2->setColor(1,1,1,.5);

    // start the simulation
    scene.startViewer();

    BulletInstance::Ptr bullet2(new BulletInstance);
    OSGInstance::Ptr osg2(new OSGInstance);
    scene.osg->root->addChild(osg2->root.get());
    Fork::Ptr fork(new Fork(scene.env, bullet2, osg2));

    while(true) {
    	fork->env->step(0.03, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
    	scene.step(0.03, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
    }

    return 0;
}
