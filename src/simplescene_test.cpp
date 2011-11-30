#include "simplescene.h"

int main() {
    Scene scene(true, true);
    scene.startViewer();
    scene.viewerLoop();
    return 0;
}
