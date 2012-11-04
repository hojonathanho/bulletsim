#include "simulation/simplescene.h"
#include "simulation/softbodies.h"
//#include "simulation/util.h"
#include "simulation/config_bullet.h"

int main (int argc, char* argv[]) {
  
    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);

    Scene s;

    std::vector<btVector3> corners;
    corners.push_back(btVector3(0,0,0.1));
    corners.push_back(btVector3(0.5,0,0.1));
    corners.push_back(btVector3(0.5,0.5,0.1));
    corners.push_back(btVector3(0,0.5,0.1));
    BulletSoftObject::Ptr sbp = makeSponge(corners,0.1,3);
    sbp->setColor(1,1,1,1);
    sbp->softBody->translate(btVector3(0,0,0.01*METERS));
    //    TrackedSponge::Ptr tracked_sponge(new TrackedSponge(sbp));

    s.env->add(sbp);
        s.startViewer();
        s.startLoop();
    return 0;
}
