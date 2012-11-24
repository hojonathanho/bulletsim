#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include <boost/thread/thread.hpp>

int main(int argc, char *argv[]) {

    SceneConfig::enableIK = false;
    SceneConfig::enableHaptics = false;
    SceneConfig::enableRobot = false;

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SceneConfig());
  parser.read(argc, argv);


    Scene scene;

    SphereObject::Ptr sphere(new SphereObject(1, 0.1, btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 10))));
    scene.env->add(sphere);

    BoxObject::Ptr box(new BoxObject(0.1, btVector3(0.1, 0.1, 0.1), btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0.5, 1))));
    scene.env->add(box);

    box->rigidBody->setCenterOfMassTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(1, 0.5, 1)));

    ObjectAction::Ptr moveSphere = sphere->createMoveAction(
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
