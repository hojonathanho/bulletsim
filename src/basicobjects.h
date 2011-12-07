#ifndef _BASICOBJECTS_H_
#define _BASICOBJECTS_H_

#include "environment.h"

class GrabberKinematicObject : public BulletKinematicObject {
private:
    float radius, height;
    btVector3 constraintPivot;
    boost::shared_ptr<btGeneric6DofConstraint> constraint;

public:
    typedef boost::shared_ptr<GrabberKinematicObject> Ptr;

    GrabberKinematicObject(float radius_, float height_);
    EnvironmentObject::Ptr copy() { return Ptr(new GrabberKinematicObject(*this)); }
    void destroy() { releaseConstraint(); BulletKinematicObject::destroy(); }

    osg::ref_ptr<osg::Node> createOSGNode();

    void grabNearestObjectAhead();
    void releaseConstraint();
};

// An infinite surface on the X-Y plane.
// the drawHalfExtents argument to the constructor is for rendering only
class PlaneStaticObject : public BulletObject {
private:
    const btVector3 planeNormal;
    const btScalar planeConstant;
    const btScalar drawHalfExtents;

public:
    typedef boost::shared_ptr<PlaneStaticObject> Ptr;

    PlaneStaticObject(const btVector3 &planeNormal_, btScalar planeConstant_,
                      boost::shared_ptr<btDefaultMotionState> motionState_, btScalar drawHalfExtents_=50.);
    EnvironmentObject::Ptr copy() { return Ptr(new PlaneStaticObject(*this)); }

    // must override this since osgBullet doesn't recognize btStaticPlaneShape
    osg::ref_ptr<osg::Node> createOSGNode();
};

// A cylinder along the Z-axis
class CylinderStaticObject : public BulletObject {
private:
    btScalar mass, radius, height;

public:
    typedef boost::shared_ptr<CylinderStaticObject> Ptr;

    CylinderStaticObject(btScalar mass_, btScalar radius_, btScalar height_,
                         boost::shared_ptr<btDefaultMotionState> motionState_);
    EnvironmentObject::Ptr copy() { return Ptr(new CylinderStaticObject(*this)); }
};

class SphereObject : public BulletObject {
private:
    btScalar mass, radius;

public:
    typedef boost::shared_ptr<SphereObject> Ptr;

    SphereObject(btScalar mass_, btScalar radius_,
                 boost::shared_ptr<btDefaultMotionState> motionState_);
    EnvironmentObject::Ptr copy() { return Ptr(new SphereObject(*this)); }
};


class BoxObject : public BulletObject {
private:
  btScalar mass;
  btVector3 halfExtents;

public:
    typedef boost::shared_ptr<BoxObject> Ptr;

    BoxObject(btScalar mass_, btVector3 halfExtents_,
                 boost::shared_ptr<btDefaultMotionState> motionState_);
    EnvironmentObject::Ptr copy() { return Ptr(new BoxObject(*this)); }
};

// A wrapper for btCapsuleShapeX
class CapsuleObject : public BulletObject {
private: 
    btScalar mass, radius, height;

public:
    typedef boost::shared_ptr<CapsuleObject> Ptr;

    CapsuleObject(btScalar mass_, btScalar radius_, btScalar height_,
                  boost::shared_ptr<btDefaultMotionState> motionState_);
    EnvironmentObject::Ptr copy() { return Ptr(new CapsuleObject(*this)); }
    osg::ref_ptr<osg::Node> createOSGNode();
};

#endif // _BASICOBJECTS_H_
