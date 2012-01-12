#include "basescene.h"
#include "openravesupport.h"

int main(int argc, char *argv[]) {
    typedef boost::shared_ptr<BaseScene> Ptr;
    // construct the scene
    BaseScene scene;
    RaveInstance::Ptr rave;
    rave.reset(new RaveInstance());
    RaveRobotKinematicObject::Ptr pr2;
    btTransform trans(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0));
    pr2.reset(new RaveRobotKinematicObject(rave, "robots/puma.robot.xml", trans, GeneralConfig::scale));
    scene.env->add(pr2);
    // manipulate the scene or add more objects, if desired
    boost::shared_ptr<btDefaultMotionState> ms;
    ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 10))));
    boost::shared_ptr<btDefaultMotionState> ms2;
    ms2.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(1, 0, 10))));
    BoxObject::Ptr box(new BoxObject(0.1, btVector3(0.1, 0.1, 0.1), ms2));
    scene.env->add(box);

    SphereObject::Ptr sphere(new SphereObject(1, 0.1, ms));
    scene.env->add(sphere);

    // start the simulation
    scene.startViewer();
    scene.startLoop();

    return 0;
}
