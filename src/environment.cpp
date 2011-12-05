#include "environment.h"
#include <osgbCollision/CollisionShapes.h>
#include <Serialize/BulletFileLoader/btBulletFile.h>

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
    return (i == objMap.end()) ? EnvironmentObject::Ptr() : i->second;
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

EnvironmentObject::Ptr BulletObject::copy() {
    // we need to access lots of private members of btRigidBody and etc
    // the easiest way to do this is to use serialization
    boost::shared_ptr<btDefaultSerializer> serializer(new btDefaultSerializer());
    int len; btChunk *chunk; const char *structType;
    // http://www.bulletphysics.com/Bullet/BulletFull/btDiscreteDynamicsWorld_8cpp_source.html#l01147
    serializer->startSerialization();
        // rigid body
        len = rigidBody->calculateSerializeBufferSize();
        chunk = serializer->allocate(len, 1);
        structType = rigidBody->serialize(chunk->m_oldPtr, serializer.get());
        serializer->finalizeChunk(chunk, structType, BT_RIGIDBODY_CODE, rigidBody.get());
        // collision shape
//        len = collisionShape->calculateSerializeBufferSize();
//        chunk = serializer->allocate(len, 1);
//        structType = collisionShape->serialize(chunk->m_oldPtr, serializer.get());
//        serializer->finalizeChunk(chunk, structType, BT_SHAPE_CODE, collisionShape);
        // TODO: constraints?
    serializer->finishSerialization();

    // load everything back
    boost::shared_ptr<btRigidBody> newRigidBody;

    // just use the old collisionShape. yes this isn't strictly a deep copy
    boost::shared_ptr<btCollisionShape> newCollisionShape(collisionShape);

    int bufSize = serializer->getCurrentBufferSize();
    char *buf = new char[bufSize];
    memcpy(buf, serializer->getBufferPointer(), bufSize);
    boost::shared_ptr<bParse::btBulletFile> bulletFile(new bParse::btBulletFile(
                buf, bufSize));
//                serializer->getBufferPointer(), serializer->getCurrentBufferSize()));
    if (bulletFile->getFlags() & bParse::FD_DOUBLE_PRECISION) {
    } else {
        btRigidBodyFloatData *colObjData = reinterpret_cast<btRigidBodyFloatData *> (bulletFile->m_rigidBodies[0]);
        btScalar mass = btScalar(colObjData->m_inverseMass? 1.f/colObjData->m_inverseMass : 0.f);
        btVector3 localInertia; localInertia.setZero();
//        btCollisionShape** shapePtr = m_data2shapeMap.find(colObjData->m_collisionObjectData.m_collisionShape);
//        btCollisionShape** shapePtr = &collisionShape.get();
//        if (shapePtr && *shapePtr) {
            btTransform startTransform;
            startTransform.deSerializeFloat(colObjData->m_collisionObjectData.m_worldTransform);
//	startTransform.setBasis(btMatrix3x3::getIdentity());
            btCollisionShape* shape = newCollisionShape.get();
            if (shape->isNonMoving())
		mass = 0.f;
            if (mass)
                shape->calculateLocalInertia(mass,localInertia);
            bool isDynamic = mass!=0.f;
            btRigidBody* body = rigidBodyFromData(mass, colObjData, startTransform, shape);
            if (body) {
                printf("body was created");
//                if (m_dynamicsWorld)
//                        m_dynamicsWorld->addRigidBody(body);
                //spawnResult->m_allocatedRigidBodies.push_back(body);
                newRigidBody.reset(body);
//                spawnResult->m_allocatedRigidBodyNames.push_back(colObjData->m_collisionObjectData.m_name);
            }
#if 0
	#ifdef USE_INTERNAL_EDGE_UTILITY
					if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
					{
						btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*)shape;
						if (trimesh->getTriangleInfoMap())
						{
							body->setCollisionFlags(body->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
						}
					}
	#endif //USE_INTERNAL_EDGE_UTILITY
#endif
                //bodyMap.insert(colObjData,body);
//            } else
//                    printf("error: no shape found\n");
    }
    delete [] buf;




//    boost::shared_ptr<btBulletWorldImporter> importer(new btBulletWorldImporter(0));
//    importer->loadFileFromMemory(serializer->getBufferPointer(), serializer->getCurrentBufferSize());
//    BOOST_ASSERT(importer->getNumRigidBodies() == 1);
//    boost::shared_ptr<btRigidBody> newRigidBody(importer->getRigidBodyByIndex(0));

    // duplicate the bullet object (this is the hard part)

    // to get the new motionstate, just do a direct memory copy of the current one
    // (this should be safe)
    //char *buf = new char[sizeof(*motionState)];
    boost::shared_ptr<btMotionState> newMotionState();
    //memcpy(newMotionState);

    return new BulletObject(newCollisionShape, newRigidBody, newMotionState);
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
