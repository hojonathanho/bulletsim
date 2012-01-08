#include "environment.h"
#include "SetColorsVisitor.h"

OSGInstance::OSGInstance() {
    root = new osg::Group;
}

BulletInstance::BulletInstance() {
    broadphase = new btDbvtBroadphase();
    collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    solver = new btSequentialImpulseConstraintSolver;
    dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    dynamicsWorld->getDispatchInfo().m_enableSPU = true;

    softBodyWorldInfo.m_broadphase = broadphase;
    softBodyWorldInfo.m_dispatcher = dispatcher;
    softBodyWorldInfo.m_sparsesdf.Initialize();
}

BulletInstance::~BulletInstance() {
    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete broadphase;
}

void BulletInstance::setGravity(const btVector3 &gravity) {
    dynamicsWorld->setGravity(gravity);
    softBodyWorldInfo.m_gravity = gravity;
}

void BulletInstance::contactTest(btCollisionObject *obj,
                                CollisionObjectSet &out,
                                const CollisionObjectSet *ignore) {
    struct ContactCallback : public btCollisionWorld::ContactResultCallback {
        const CollisionObjectSet *ignore;
        CollisionObjectSet &out;
        ContactCallback(const CollisionObjectSet *ignore_, CollisionObjectSet &out_) :
            ignore(ignore_), out(out_) { }
        btScalar addSingleResult(btManifoldPoint &,
                                 const btCollisionObject *colObj0, int, int,
                                 const btCollisionObject *colObj1, int, int) {
            if (ignore && ignore->find(colObj1) == ignore->end())
                out.insert(colObj1);
            return 0;
        }
    } cb(ignore, out);
    dynamicsWorld->contactTest(obj, cb);
}

Environment::~Environment() {
    for (ObjectList::iterator i = objects.begin(); i != objects.end(); ++i)
        (*i)->destroy();
}

void Environment::add(EnvironmentObject::Ptr obj) {
    obj->setEnvironment(this);
    obj->init();
    objects.push_back(obj);
    // objects are reponsible for adding themselves
    // to the dynamics world and the osg root
}

void Environment::step(btScalar dt, int maxSubSteps, btScalar fixedTimeStep) {
    ObjectList::iterator i;
    for (i = objects.begin(); i != objects.end(); ++i)
        (*i)->prePhysics();
    bullet->dynamicsWorld->stepSimulation(dt, maxSubSteps, fixedTimeStep);
    for (i = objects.begin(); i != objects.end(); ++i)
        (*i)->preDraw();
    bullet->softBodyWorldInfo.m_sparsesdf.GarbageCollect();
}

Environment::Fork::Fork(Environment *parentEnv_, BulletInstance::Ptr bullet, OSGInstance::Ptr osg) :
    parentEnv(parentEnv_), env(new Environment(bullet, osg)) { }

EnvironmentObject::Ptr Environment::Fork::forkOf(EnvironmentObject::Ptr orig) {
    ObjectMap::iterator i = objMap.find(orig);
    return i == objMap.end() ? EnvironmentObject::Ptr() : i->second;
}

Environment::Fork::Ptr Environment::fork(BulletInstance::Ptr newBullet, OSGInstance::Ptr newOSG) {
    Fork::Ptr f(new Fork(this, newBullet, newOSG));
    for (ObjectList::const_iterator i = objects.begin(); i != objects.end(); ++i) {
        EnvironmentObject::Ptr copy = (*i)->copy();
        f->env->add(copy);
        f->objMap[*i] = copy;
    }
    return f;
}
