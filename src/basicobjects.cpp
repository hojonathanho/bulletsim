#include "basicobjects.h"
#include "config_bullet.h"
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osgwTools/Shapes.h>
#include <osgbCollision/CollisionShapes.h>
#include <Serialize/BulletFileLoader/btBulletFile.h>
#include <boost/scoped_array.hpp>
#include "SetColorsVisitor.h"

#include <osg/BlendFunc>
#include <osg/AlphaFunc>


#define MAX_RAYCAST_DISTANCE 100.0

btRigidBody::btRigidBodyConstructionInfo getCI(btScalar mass, boost::shared_ptr<btCollisionShape> collisionShape, boost::shared_ptr<btDefaultMotionState> motionState) {
  btVector3 inertia(0,0,0);
  collisionShape->calculateLocalInertia(mass,inertia);
  btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState.get(),
					      collisionShape.get(), inertia);
  ci.m_restitution = BulletConfig::restitution;
  ci.m_friction = BulletConfig::friction;
  return ci;
}

void BulletObject::init() {
    getEnvironment()->bullet->dynamicsWorld->addRigidBody(rigidBody.get());
    node = createOSGNode();
    transform = new osg::MatrixTransform;
    transform->addChild(node.get());
    getEnvironment()->osg->root->addChild(transform.get());

    osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
    blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    osg::StateSet *ss = node->getOrCreateStateSet();
    ss->setAttributeAndModes(blendFunc);
    setColorAfterInit();
}

osg::ref_ptr<osg::Node> BulletObject::createOSGNode() {
    return osg::ref_ptr<osg::Node>(osgbCollision::osgNodeFromBtCollisionShape(collisionShape.get()));
}

btScalar IDENTITY[] = {1,0,0,0,
		       0,1,0,0,
		       0,0,1,0,
		       0,0,0,1};

btScalar* identityIfBad(btScalar m[16]) { //doesn't fix segfaults, unfortunately.
  for (int i=0; i<16; i++) {
    if (m[i] > 1000 || m[i] < -1000 || !isfinite(m[i])) {
      cout << "warning: bad values detected in transformation matrix. rendering at origin" << endl;
      return IDENTITY;
    }
  }
  return m;
}

void BulletObject::preDraw() {
    // before drawing, we must copy the orientation/position
    // of the object from Bullet to OSG
    btTransform btTrans;
    rigidBody->getMotionState()->getWorldTransform(btTrans);

    btScalar m[16];
    btTrans.getOpenGLMatrix(m);
    transform->setMatrix(osg::Matrix(identityIfBad(m)));
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
    boost::shared_ptr<bParse::btBulletFile> bulletFile(
        new bParse::btBulletFile(buf.get(), bufSize));
    bulletFile->parse(false);
    // create a new rigidBody with the data
    printf("num rigidbodies: %d\n", bulletFile->m_rigidBodies.size());
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

        // extra "active" params
        rigidBody->setLinearVelocity(o.rigidBody->getLinearVelocity());
        rigidBody->setAngularVelocity(o.rigidBody->getAngularVelocity());
        rigidBody->applyCentralForce(o.rigidBody->getTotalForce());
        rigidBody->applyTorque(o.rigidBody->getTotalTorque());

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

void BulletObject::MoveAction::step(float dt) {
    if (done()) return;
    stepTime(dt);
    const float a = fracElapsed();

    // linear interpolation of pos
    btVector3 newpos = (1-a)*start.getOrigin() + a*end.getOrigin();
    btQuaternion newrot = start.getRotation().slerp(end.getRotation(), a);
    btTransform newtrans(newrot, newpos);
    obj->motionState->setWorldTransform(newtrans);
}

void BulletObject::setColor(float r, float g, float b, float a) {
  m_color.reset(new osg::Vec4f(r,g,b,a));
  if (node) setColorAfterInit();
}
void BulletObject::setColorAfterInit() {
  if (m_color) {
    if (m_color->a() != 1.0f) {
      osg::StateSet *ss = node->getOrCreateStateSet();
      ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    }

    SetColorsVisitor visitor(m_color->r(),m_color->g(),m_color->b(),m_color->a());
    node->accept(visitor);
  }
}

BulletKinematicObject::BulletKinematicObject(boost::shared_ptr<btCollisionShape> collisionShape_, const btTransform &trans) {
    collisionShape = collisionShape_;
    motionState.reset(new MotionState(this, trans));
    
    // (the collisionShape is set by the constructor)
    // all kinematic objects have zero mass and inertia
    btRigidBody::btRigidBodyConstructionInfo ci(0., motionState.get(), collisionShape.get(), btVector3(0., 0., 0.));
    rigidBody.reset(new btRigidBody(ci));

    // special flags for kinematic objects
    rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    rigidBody->setActivationState(DISABLE_DEACTIVATION);
}


GrabberKinematicObject::GrabberKinematicObject(float radius_, float height_) :
    radius(radius_), height(height_),
    constraintPivot(0, 0, height), // this is where objects will attach to
    BulletKinematicObject(boost::shared_ptr<btCollisionShape>(new btConeShapeZ(radius, height)),
        btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0))) {
}

osg::ref_ptr<osg::Node> GrabberKinematicObject::createOSGNode() {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Cone> cone = new osg::Cone(osg::Vec3(), radius, height);
    cone->setCenter(osg::Vec3(0, 0, -cone->getBaseOffset()));
    // FIXME: this cone seems a bit taller than the bullet cone
    geode->addDrawable(new osg::ShapeDrawable(cone));
    return geode;
}

void GrabberKinematicObject::grabNearestObjectAhead() {
    // first, try to find the object ahead.
    // trace a ray in the direction of the end affector and get the first object hit
    btTransform trans; motionState->getWorldTransform(trans);
    btVector3 rayFrom = trans(btVector3(0, 0, 0)); // some point in the middle of the stick
    btVector3 rayTo = trans(constraintPivot); // the end affector
    rayTo = (rayTo - rayFrom).normalize()*MAX_RAYCAST_DISTANCE + rayTo;

    printf("from: %f %f %f\nto:%f %f %f\n", rayFrom.x(), rayFrom.y(), rayFrom.z(), rayTo.x(), rayTo.y(), rayTo.z());
    // send the ray
    btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom, rayTo);
    getEnvironment()->bullet->dynamicsWorld->rayTest(rayFrom, rayTo, rayCallback);
    printf("tracing!\n");
    if (rayCallback.hasHit()) {
        printf("hit!\n");
        btRigidBody *hitBody = btRigidBody::upcast(rayCallback.m_collisionObject);
        if (hitBody && !hitBody->isStaticObject() && !hitBody->isKinematicObject()) {
            hitBody->setActivationState(DISABLE_DEACTIVATION);

            releaseConstraint();

            // the constraint in the grabber's coordinate system
            btTransform grabberFrame;
            grabberFrame.setIdentity();
            grabberFrame.setOrigin(constraintPivot);
            grabberFrame.setRotation(trans.inverse().getRotation());

            // the constraint in the target's coordinate system
            const btTransform &hitBodyTransInverse = hitBody->getCenterOfMassTransform().inverse();
            btVector3 localHitPoint = hitBodyTransInverse * rayCallback.m_hitPointWorld;
            btTransform hitFrame;
            hitFrame.setIdentity();
            hitFrame.setOrigin(localHitPoint);
            hitFrame.setRotation(hitBodyTransInverse.getRotation());

            constraint.reset(new btGeneric6DofConstraint(*rigidBody, *hitBody, grabberFrame, hitFrame, false));
            // make the constraint completely rigid
            constraint->setLinearLowerLimit(btVector3(0., 0., 0.));
            constraint->setLinearUpperLimit(btVector3(0., 0., 0.));
            constraint->setAngularLowerLimit(btVector3(0., 0., 0.));
            constraint->setAngularUpperLimit(btVector3(0., 0., 0.));

            getEnvironment()->bullet->dynamicsWorld->addConstraint(constraint.get());
        }
    }
}

void GrabberKinematicObject::releaseConstraint() {
    if (!constraint) return;
    getEnvironment()->bullet->dynamicsWorld->removeConstraint(constraint.get());
    constraint.reset();
}


PlaneStaticObject::PlaneStaticObject(
        const btVector3 &planeNormal_,
        btScalar planeConstant_,
        boost::shared_ptr<btDefaultMotionState> motionState_, btScalar drawHalfExtents_) :
            planeNormal(planeNormal_), planeConstant(planeConstant_), drawHalfExtents(drawHalfExtents_) {
    motionState = motionState_;
    collisionShape.reset(new btStaticPlaneShape(planeNormal, planeConstant));
    btRigidBody::btRigidBodyConstructionInfo ci(0., motionState.get(), collisionShape.get(), btVector3(0., 0., 0.));
    rigidBody.reset(new btRigidBody(ci));
}

osg::ref_ptr<osg::Node> PlaneStaticObject::createOSGNode() {
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(-drawHalfExtents, -drawHalfExtents, 0.));
    vertices->push_back(osg::Vec3(drawHalfExtents, -drawHalfExtents, 0.));
    vertices->push_back(osg::Vec3(drawHalfExtents, drawHalfExtents, 0.));
    vertices->push_back(osg::Vec3(-drawHalfExtents, drawHalfExtents, 0.));

    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0., 0., 1.));

    osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;
    quad->setVertexArray(vertices.get());
    quad->setNormalArray(normals.get());
    quad->setNormalBinding(osg::Geometry::BIND_OVERALL);
    quad->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(quad.get());
    return geode;
}

CylinderStaticObject::CylinderStaticObject(
        btScalar mass_, btScalar radius_, btScalar height_,
        boost::shared_ptr<btDefaultMotionState> motionState_) :
            mass(mass_), radius(radius_), height(height_) {
    motionState = motionState_;
    collisionShape.reset(new btCylinderShapeZ(btVector3(radius, radius, height/2.)));
    btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState.get(),
                                                collisionShape.get(), btVector3(0., 0., 0.));
    rigidBody.reset(new btRigidBody(ci));
}

SphereObject::SphereObject(btScalar mass_, btScalar radius_, boost::shared_ptr<btDefaultMotionState> motionState_) :
        mass(mass_), radius(radius_) {
    motionState = motionState_; 
    collisionShape.reset(new btSphereShape(radius));
    btVector3 fallInertia(0, 0, 0);
    collisionShape->calculateLocalInertia(mass, fallInertia);
    btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState.get(),
                                                collisionShape.get(), fallInertia);
    rigidBody.reset(new btRigidBody(ci));
    rigidBody->setActivationState(DISABLE_DEACTIVATION);
}


BoxObject::BoxObject(btScalar mass_, btVector3 halfExtents_, boost::shared_ptr<btDefaultMotionState> motionState_) :
  mass(mass_), halfExtents(halfExtents_) {
  motionState = motionState_;
  collisionShape.reset(new btBoxShape(halfExtents));
  btVector3 fallInertia(0,0,0);
  collisionShape->calculateLocalInertia(mass,fallInertia);
  btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState.get(),
					      collisionShape.get(), fallInertia);
  rigidBody.reset(new btRigidBody(ci));
  rigidBody->setActivationState(DISABLE_DEACTIVATION);
}


CapsuleObject::CapsuleObject(btScalar mass_, btScalar radius_, btScalar height_,
          boost::shared_ptr<btDefaultMotionState> motionState_) : mass(mass_), radius(radius_), height(height_) {
    motionState = motionState_;
    collisionShape.reset(new btCapsuleShapeX(radius, height));
    btVector3 fallInertia(0, 0, 0);
    collisionShape->calculateLocalInertia(mass, fallInertia);
    btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState.get(),
                                                collisionShape.get(), fallInertia);
    rigidBody.reset(new btRigidBody(ci));
    rigidBody->setActivationState(DISABLE_DEACTIVATION);
}

osg::ref_ptr<osg::Node> CapsuleObject::createOSGNode() {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Capsule> capsule = new osg::Capsule(osg::Vec3(0, 0, 0), radius, height);
    capsule->setRotation(osg::Quat(osg::PI_2, osg::Vec3(0, 1, 0)));
    geode->addDrawable(new osg::ShapeDrawable(capsule));
    return geode;
}
