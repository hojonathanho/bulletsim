#include "basicobjects.h"
#include <osg/Geometry>
#include <osg/Geode>

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
}
