#include "emptyscene.h"

int main(int argc, char *argv[]) {
    // construct the scene
    EmptyScene scene;
    // manipulate the scene or add more objects, if desired

    // start the simulation
    scene.startViewer();
    scene.startLoop();

    return 0;
}
