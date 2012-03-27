#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "simulation/plotting.h"
#include "robots/pr2.h"
#include "clothgrasping.h"

static void openGripper(Scene *scene, GripperOpenCloseAction *a, bool open) {
    a->reset();
    a->setOpen(open);
    scene->runAction(*a, BulletConfig::dt);
}

static void drawGripperRegion(Scene *scene, PlotBoxes::Ptr plot, PR2Manager *pr2m) {
}

int main(int argc, char *argv[]) {
    SceneConfig::enableIK = true;
    SceneConfig::enableRobot = true;
    SceneConfig::enableHaptics = false;

    Parser parser;
    parser.addGroup(GeneralConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);

    Scene scene;

    PlotBoxes::Ptr boxplot(new PlotBoxes);
    scene.env->add(boxplot);

    PR2Manager pr2m(scene);
    GripperOpenCloseAction leftA(pr2m.pr2, pr2m.pr2Left->manip, true);
    GripperOpenCloseAction rightA(pr2m.pr2, pr2m.pr2Right->manip, true);
    scene.addVoidKeyCallback('a', boost::bind(openGripper, &scene, &leftA, true));
    scene.addVoidKeyCallback('z', boost::bind(openGripper, &scene, &leftA, false));
    scene.addVoidKeyCallback('s', boost::bind(openGripper, &scene, &rightA, true));
    scene.addVoidKeyCallback('x', boost::bind(openGripper, &scene, &rightA, false));

//    scene.addPreDrawCallback(boost::bind(drawGripperRegion, &scene, boxplot, &pr2m));

    boxplot->addBox(osg::Vec3(1, 1, 1), 1, 2, 3, osg::Vec4(1, 0, 0, 0.5));

    scene.startViewer();
    scene.startLoop();

    return 0;
}
