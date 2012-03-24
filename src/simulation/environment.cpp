#include "environment.h"
#include "utils/config.h"

OSGInstance::OSGInstance() {
    root = new osg::Group;
}

BulletInstance::BulletInstance() {
  broadphase = new btDbvtBroadphase();
  //    broadphase = new btAxisSweep3(btVector3(-2*METERS, -2*METERS, -1*METERS), btVector3(2*METERS, 2*METERS, 3*METERS));
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
    for (ConstraintList::iterator i = constraints.begin(); i != constraints.end(); ++i)
        (*i)->destroy();
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

void Environment::remove(EnvironmentObject::Ptr obj) {
    for (ObjectList::iterator i = objects.begin(); i != objects.end(); ++i) {
        if (obj == *i) {
            (*i)->destroy();
            objects.erase(i);
            return;
        }
    }
}

void Environment::addConstraint(EnvironmentObject::Ptr cnt) {
    cnt->setEnvironment(this);
    cnt->init();
    constraints.push_back(cnt);
}

void Environment::removeConstraint(EnvironmentObject::Ptr cnt) {
    for (ConstraintList::iterator i = constraints.begin(); i != constraints.end(); ++i) {
        if (cnt == *i) {
            (*i)->destroy();
            constraints.erase(i);
            return;
        }
    }
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

void Fork::copyObjects() {
    // copy objects first
    Environment::ObjectList::const_iterator i;
    for (i = parentEnv->objects.begin(); i != parentEnv->objects.end(); ++i) {
        EnvironmentObject::Ptr copy = (*i)->copy(*this);
        env->add(copy);
        objMap[*i] = copy;
    }
    // some objects might need processing after all objects have been added
    // e.g. anchors and joints for soft bodies
    for (i = parentEnv->objects.begin(); i != parentEnv->objects.end(); ++i)
        (*i)->postCopy(objMap[*i], *this);

    // copy constraints
    Environment::ConstraintList::const_iterator j;
    for (j = parentEnv->constraints.begin(); j != parentEnv->constraints.end(); ++j) {
        EnvironmentObject::Ptr copy = (*j)->copy(*this);
        env->addConstraint(copy);
        objMap[*j] = copy;
    }
    for (j = parentEnv->constraints.begin(); j != parentEnv->constraints.end(); ++j)
        (*j)->postCopy(objMap[*j], *this);
}
