#include "simulation/scenecontrol.h"

int main(int argc, char *argv[]) {
    SceneControl controller;

    // start the simulation
    controller.scene.startViewer();
    controller.scene.startLoop();

    return 0;
}
