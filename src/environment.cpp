#include "environment.h"
#include <osgbCollision/CollisionShapes.h>
#include <Serialize/BulletFileLoader/btBulletFile.h>
#include <boost/scoped_array.hpp>

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

EnvironmentObject::Ptr Environment::Fork::correspondingObject(EnvironmentObject::Ptr orig) {
    ObjectMap::iterator i = objMap.find(orig);
    return i == objMap.end() ? EnvironmentObject::Ptr() : i->second;
}

Environment::Fork::Ptr Environment::fork(BulletInstance::Ptr newBullet) {
    Fork::Ptr f(new Fork(this, newBullet, osg));
    for (ObjectList::const_iterator i = objects.begin(); i != objects.end(); ++i) {
        EnvironmentObject::Ptr copy = (*i)->copy();
        f->env->add(copy);
        f->objMap[*i] = copy;
    }
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

BulletObject::BulletObject(const BulletObject &o) {
    // we need to access lots of private members of btRigidBody and etc
    // the easiest way to do this is to use serialization

    // first copy over the collisionShape. This isn't a real deep copy,
    // but we can share collisionShapes so this should be fine
    collisionShape = o.collisionShape;

    // then copy the motionstate
    motionState.reset(new btDefaultMotionState(*o.motionState.get()));

    // then serialize the rigid body
    boost::shared_ptr<btDefaultSerializer> serializer(new btDefaultSerializer());
    int len; btChunk *chunk; const char *structType;
    // http://www.bulletphysics.com/Bullet/BulletFull/btDiscreteDynamicsWorld_8cpp_source.html#l01147
    serializer->startSerialization();
        // rigid body
        len = o.rigidBody->calculateSerializeBufferSize();
        chunk = serializer->allocate(len, 1);
        structType = o.rigidBody->serialize(chunk->m_oldPtr, serializer.get());
        serializer->finalizeChunk(chunk, structType, BT_RIGIDBODY_CODE, o.rigidBody.get());
        // collision shape
//        len = collisionShape->calculateSerializeBufferSize();
//        chunk = serializer->allocate(len, 1);
//        structType = collisionShape->serialize(chunk->m_oldPtr, serializer.get());
//        serializer->finalizeChunk(chunk, structType, BT_SHAPE_CODE, collisionShape);
        // TODO: constraints?
    serializer->finishSerialization();

    // read the data that the serializer just wrote
    int bufSize = serializer->getCurrentBufferSize();
    boost::scoped_array<char> buf(new char[bufSize]);
    memcpy(buf.get(), serializer->getBufferPointer(), bufSize);
    boost::shared_ptr<bParse::btBulletFile> bulletFile(new bParse::btBulletFile(
                buf.get(), bufSize));
    // create a new rigidBody with the data
    BOOST_ASSERT(bulletFile->m_rigidBodies.size() == 1);
    if (bulletFile->getFlags() & bParse::FD_DOUBLE_PRECISION) {
        // double precision not supported
        BOOST_ASSERT(false);
    } else {
        // single precision
        btRigidBodyFloatData *data = reinterpret_cast<btRigidBodyFloatData *> (bulletFile->m_rigidBodies[0]);
        btScalar mass = btScalar(data->m_inverseMass? 1.f/data->m_inverseMass : 0.f);
        btVector3 localInertia; localInertia.setZero();
        btTransform startTransform;
        startTransform.deSerializeFloat(data->m_collisionObjectData.m_worldTransform);
        //	startTransform.setBasis(btMatrix3x3::getIdentity());
        if (collisionShape->isNonMoving())
            mass = 0.f;
        if (mass)
            collisionShape->calculateLocalInertia(mass, localInertia);

        // fill in btRigidBody params
        btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState.get(), collisionShape.get(), localInertia);
        ci.m_linearDamping = data->m_linearDamping;
        ci.m_angularDamping = data->m_angularDamping;
        ci.m_additionalDampingFactor = data->m_additionalDampingFactor;
        ci.m_additionalLinearDampingThresholdSqr = data->m_additionalLinearDampingThresholdSqr;
        ci.m_additionalAngularDampingThresholdSqr = data->m_additionalAngularDampingThresholdSqr;
        ci.m_additionalAngularDampingFactor = data->m_additionalAngularDampingFactor;
        ci.m_linearSleepingThreshold = data->m_linearSleepingThreshold;
        ci.m_angularSleepingThreshold = data->m_angularSleepingThreshold;
        ci.m_additionalDamping = data->m_additionalDamping;
        rigidBody.reset(new btRigidBody(ci));

        // fill in btCollisionObject params for the rigid body
        btCollisionObject *colObj = rigidBody.get();
        btCollisionObjectFloatData &colObjData = data->m_collisionObjectData;
        btVector3 temp;
        temp.deSerializeFloat(colObjData.m_anisotropicFriction);
        colObj->setAnisotropicFriction(temp);
        colObj->setContactProcessingThreshold(colObjData.m_contactProcessingThreshold);
        colObj->setFriction(colObjData.m_friction);
        colObj->setRestitution(colObjData.m_restitution);
        colObj->setCollisionFlags(colObjData.m_collisionFlags);
        colObj->setHitFraction(colObjData.m_hitFraction);
        // TODO: activation state
    }
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
