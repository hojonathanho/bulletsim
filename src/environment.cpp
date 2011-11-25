#include "environment.h"
#include <osgbCollision/CollisionShapes.h>
#include <osgUtil/SmoothingVisitor>
#include <iostream>
using namespace std;
OSGInstance::OSGInstance() {
    root = new osg::Group;
}

BulletInstance::BulletInstance() {
    broadphase = new btDbvtBroadphase();
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    solver = new btSequentialImpulseConstraintSolver;
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
}

BulletInstance::~BulletInstance() {
    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete broadphase;
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
}

void BulletObject::init() {
    getEnvironment()->bullet->dynamicsWorld->addRigidBody(rigidBody.get());
    node = createOSGNode();
    transform = new osg::MatrixTransform;
    transform->addChild(node.get());
    getEnvironment()->osg->root->addChild(transform.get());
}

osg::ref_ptr<osg::Node> BulletObject::createOSGNode() {
    return osg::ref_ptr<osg::Node>(osgbCollision::osgNodeFromBtCollisionShape(collisionShape.get()));
}

void BulletObject::preDraw() {
    // before drawing, we must copy the orientation/position
    // of the object from Bullet to OSG
    btTransform btTrans;
    rigidBody->getMotionState()->getWorldTransform(btTrans);

    btScalar m[16];
    btTrans.getOpenGLMatrix(m);

    transform->setMatrix(osg::Matrix(m));
}

void BulletObject::destroy() {
    getEnvironment()->bullet->dynamicsWorld->removeRigidBody(rigidBody.get());
}

BulletKinematicObject::BulletKinematicObject(boost::shared_ptr<btCollisionShape> collisionShape_, const btTransform &trans) {
    collisionShape = collisionShape_;
    motionState.reset(new MotionState(trans));
    
    // (the collisionShape is set by the constructor)
    // all kinematic objects have zero mass and inertia
    btRigidBody::btRigidBodyConstructionInfo ci(0., motionState.get(), collisionShape.get(), btVector3(0., 0., 0.));
    rigidBody.reset(new btRigidBody(ci));

    // special flags for kinematic objects
    rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    rigidBody->setActivationState(DISABLE_DEACTIVATION);
}
