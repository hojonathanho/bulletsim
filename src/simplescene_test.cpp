#include "simplescene.h"
#include "userconfig.h"

int main(int argc, char *argv[]) {
    // first read the configuration from the user
    CFG->read(argc, argv);

    // and override config values to what we want
    CFG->scene.enableIK = true;
    CFG->scene.enableRobot = true;
    CFG->scene.enableHaptics = true;

    // construct the scene
    Scene scene;
    // manipulate the scene or add more objects, if desired

    // start the simulation
    scene.startViewer();
    scene.viewerLoop();

    return 0;
}
