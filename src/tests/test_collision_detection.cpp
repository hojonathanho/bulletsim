#include "simulation/environment.h"
#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include <boost/shared_ptr.hpp>

int main(int argc, char **argv) {
    SceneConfig::enableRobot = false;
    

    Parser parser;

    parser.addGroup(GeneralConfig());
    parser.addGroup(BulletConfig());
    parser.addGroup(SceneConfig());
    parser.read(argc, argv);



Scene s;
    s.bullet->setGravity(btVector3(0, 0, 0));

    boost::shared_ptr<btDefaultMotionState> ms (
            new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 1+0.01))));
    SphereObject::Ptr s1(new SphereObject(1, 1, ms));
    s.env->add(s1);

    ms.reset(new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 2+0.001, 1+0.01))));
    SphereObject::Ptr s2(new SphereObject(1, 1, ms));
    s.env->add(s2);

    BulletInstance::CollisionObjectSet ignore, out;

    ignore.insert(s.ground->rigidBody.get());

    cout << "=== first try: spheres should NOT collide ===\n";
    out.clear();
    s.bullet->dynamicsWorld->updateAabbs();
    s.bullet->contactTest(s1->rigidBody.get(), out, &ignore);
    cout << "collided with " << out.size() << " objects\n";
    if (out.size() == 1 && *out.begin() == s2->rigidBody.get())
        cout << "collided with other sphere!\n";

    cout << "=== second try: spheres SHOULD collide ===\n";
    s2->rigidBody->setCenterOfMassTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 2, 1+0.01)));
    out.clear();
    s.bullet->dynamicsWorld->updateAabbs();
    s.bullet->contactTest(s1->rigidBody.get(), out, &ignore);
    cout << "collided with " << out.size() << " objects\n";
    if (out.size() == 1 && *out.begin() == s2->rigidBody.get())
        cout << "collided with other sphere!\n";

    cout << "=== third try: spheres should NOT collide ===\n";
    s2->rigidBody->setCenterOfMassTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 2+0.00001, 1+0.01)));
    out.clear();
    s.bullet->dynamicsWorld->updateAabbs();
    s.bullet->contactTest(s1->rigidBody.get(), out, &ignore);
    cout << "collided with " << out.size() << " objects\n";
    if (out.size() == 1 && *out.begin() == s2->rigidBody.get())
        cout << "collided with other sphere!\n";
}
