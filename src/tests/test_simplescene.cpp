#include "simplescene.h"

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


    Parser().read(argc, argv);    
    
    // construct the scene
    Scene scene;
    // manipulate the scene or add more objects, if desired

    // start the simulation
    scene.startViewer();
    scene.startLoop();

    return 0;
}
