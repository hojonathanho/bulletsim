#include "simplescene.h"
#include "userconfig.h"
#include <boost/thread/thread.hpp>

int main(int argc, char *argv[]) {
    Config::read(argc, argv);
    CFG.scene.enableIK = false;
    CFG.scene.enableHaptics = false;
    CFG.scene.enableRobot = false;

    Scene scene;

    boost::shared_ptr<btDefaultMotionState> ms;
    ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 10))));
    SphereObject::Ptr sphere(new SphereObject(1, 0.1, ms));
    scene.env->add(sphere);

    ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0.5, 1))));
    BoxObject::Ptr box(new BoxObject(0.1, btVector3(0.1, 0.1, 0.1), ms));
    scene.env->add(box);

    box->rigidBody->setCenterOfMassTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(1, 0.5, 1)));

    Action::Ptr moveSphere = sphere->createMoveAction(
        btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)),
        btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 1, 1)),
        5);

    sphere->rigidBody->setCollisionFlags(sphere->rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    scene.startViewer();
    const float dt = 0.01;

    float t = scene.viewer.getFrameStamp()->getSimulationTime();
    scene.runAction(moveSphere, dt);
    float s = scene.viewer.getFrameStamp()->getSimulationTime();
    printf("took %f to move\n", s-t);

    scene.startLoop();

    return 0;
}
