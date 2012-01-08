#include "simplescene.h"
#include "config_bullet.h"
#include <boost/thread/thread.hpp>

int main(int argc, char *argv[]) {
    Parser().read(argc, argv);
    SceneConfig::enableIK = false;
    SceneConfig::enableHaptics = false;
    SceneConfig::enableRobot = false;

    Scene scene;

    boost::shared_ptr<btDefaultMotionState> ms;
    ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 10))));
    SphereObject::Ptr sphere(new SphereObject(1, 0.1, ms));
    scene.env->add(sphere);

    ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0.5, 1))));
    BoxObject::Ptr box(new BoxObject(0.1, btVector3(0.1, 0.1, 0.1), ms));
    scene.env->add(box);

    scene.startViewer();
    const float dt = 0.01; int i;
    for (i = 0; i < 100; ++i) {
        scene.step(dt);
        boost::this_thread::sleep(boost::posix_time::milliseconds(dt*1000));
    }

    // 1 second after, fork the environment and apply a force to the copied sphere

    BulletInstance::Ptr bullet2(new BulletInstance);
    bullet2->setGravity(BulletConfig::gravity);
    OSGInstance::Ptr osg2(new OSGInstance);
    scene.osg->root->addChild(osg2->root.get());
    Environment::Fork::Ptr fork = scene.env->fork(bullet2, osg2);

    SphereObject::Ptr sphere2 = boost::static_pointer_cast<SphereObject> (fork->forkOf(sphere));
    sphere2->rigidBody->applyCentralForce(btVector3(0, 50, 0));
    for ( ; i < 1000; ++i) {
        fork->env->step(dt, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
        scene.step(dt);
        boost::this_thread::sleep(boost::posix_time::milliseconds(dt*1000));
    }

    return 0;
}
