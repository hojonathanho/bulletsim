#include "basicobjects.h"
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osgwTools/Shapes.h>

#define MAX_RAYCAST_DISTANCE 100.0

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
        boost::shared_ptr<btMotionState> motionState_, btScalar drawHalfExtents_) :
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
        boost::shared_ptr<btMotionState> motionState_) :
            mass(mass_), radius(radius_), height(height_) {
    motionState = motionState_;
    collisionShape.reset(new btCylinderShapeZ(btVector3(radius, radius, height/2.)));
    btRigidBody::btRigidBodyConstructionInfo ci(mass, motionState.get(),
                                                collisionShape.get(), btVector3(0., 0., 0.));
    rigidBody.reset(new btRigidBody(ci));
}

SphereObject::SphereObject(btScalar mass_, btScalar radius_, boost::shared_ptr<btMotionState> motionState_) :
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


BoxObject::BoxObject(btScalar mass_, btVector3 halfExtents_, boost::shared_ptr<btMotionState> motionState_) :
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
          boost::shared_ptr<btMotionState> motionState_) : mass(mass_), radius(radius_), height(height_) {
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
