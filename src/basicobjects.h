#ifndef _BASICOBJECTS_H_
#define _BASICOBJECTS_H_

#include <osg/MatrixTransform>
#include "environment.h"

// an object that is entirely specified as a bullet btRigidBody
// (the OSG model will be created from the btRigidBody, and
// will be added to the scene graph and the dynamics world, respectively)
class BulletObject : public EnvironmentObject {
public:
    typedef boost::shared_ptr<BulletObject> Ptr;

    // Bullet members
    boost::shared_ptr<btRigidBody> rigidBody;
    // the motionState and collisionShape actually don't matter; the ones
    // embedded in the rigidBody are used for simulation. However,
    // placing them here will have them automatically deallocated
    // on destruction of the BulletObject
    boost::shared_ptr<btDefaultMotionState> motionState;
    boost::shared_ptr<btCollisionShape> collisionShape;

    // OSG members
    osg::ref_ptr<osg::Node> node;
    osg::ref_ptr<osg::MatrixTransform> transform;

    BulletObject() { }
    BulletObject(boost::shared_ptr<btCollisionShape> collisionShape_, boost::shared_ptr<btRigidBody> rigidBody_) :
      collisionShape(collisionShape_), rigidBody(rigidBody_), motionState(new btDefaultMotionState()) { }
    BulletObject(boost::shared_ptr<btCollisionShape> collisionShape_, boost::shared_ptr<btRigidBody> rigidBody_,
            boost::shared_ptr<btDefaultMotionState> motionState_) :
      collisionShape(collisionShape_), rigidBody(rigidBody_), motionState(motionState_) { }
    BulletObject(const BulletObject &o); // copy constructor
    virtual ~BulletObject() { }
    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new BulletObject(*this));
        internalCopy(o, f);
        return o;
    }
    void internalCopy(BulletObject::Ptr o, Fork &f) const {
        f.registerCopy(rigidBody.get(), o->rigidBody.get());
    }

    // called by Environment
    void init();
    void preDraw();
    void destroy();

    // by default uses osgBullet. Can be overridden to provide custom OSG mesh
    virtual osg::ref_ptr<osg::Node> createOSGNode();

    // actions (for the user)
    class MoveAction : public Action {
        BulletObject *obj;
        const btTransform start, end;

    public:
        typedef boost::shared_ptr<MoveAction> Ptr;
        MoveAction(BulletObject *obj_, const btTransform &start_, const btTransform &end_, float time) : obj(obj_), start(start_), end(end_), Action(time) { }
        void step(float dt);
    };
    MoveAction::Ptr createMoveAction(const btTransform &start, const btTransform &end, float time) { return MoveAction::Ptr(new MoveAction(this, start, end, time)); }
  void setColor(float r, float g, float b, float a);
private:
  boost::shared_ptr<osg::Vec4f> m_color;
  void setColorAfterInit();
};

class BulletKinematicObject : public BulletObject {
public:
    typedef boost::shared_ptr<BulletKinematicObject> Ptr;

    // this is a motion state for kinematic objects, as described at
    // http://bulletphysics.org/mediawiki-1.5.8/index.php/MotionStates
    struct MotionState : public btDefaultMotionState {
        typedef boost::shared_ptr<MotionState> Ptr;
        BulletKinematicObject *obj;
        MotionState(BulletKinematicObject *obj_, const btTransform &trans) :
            obj(obj_), btDefaultMotionState(trans) { }
        void setWorldTransform(const btTransform &) { }
        void setKinematicPos(const btTransform &pos) {
            btDefaultMotionState::setWorldTransform(pos);

            // if we want to do collision detection in between timesteps,
            // we also have to directly set this
            obj->rigidBody->setCenterOfMassTransform(pos);
        }
    };

    BulletKinematicObject(boost::shared_ptr<btCollisionShape> collisionShape_, const btTransform &trans);
    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new BulletKinematicObject(*this));
        internalCopy(o, f);
        return o;
    }

    MotionState &getKinematicMotionState() { return *static_cast<MotionState *> (motionState.get()); }
};

class GrabberKinematicObject : public BulletKinematicObject {
private:
    float radius, height;
    btVector3 constraintPivot;
    boost::shared_ptr<btGeneric6DofConstraint> constraint;

public:
    typedef boost::shared_ptr<GrabberKinematicObject> Ptr;

    GrabberKinematicObject(float radius_, float height_);
    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new GrabberKinematicObject(*this));
        internalCopy(o, f);
        return o;
    }
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
    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new PlaneStaticObject(*this));
        internalCopy(o, f);
        return o;
    }


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
    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new CylinderStaticObject(*this));
        internalCopy(o, f);
        return o;
    }
};

class SphereObject : public BulletObject {
private:
    btScalar mass, radius;

public:
    typedef boost::shared_ptr<SphereObject> Ptr;

    SphereObject(btScalar mass_, btScalar radius_,
                 boost::shared_ptr<btDefaultMotionState> motionState_);
    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new SphereObject(*this));
        internalCopy(o, f);
        return o;
    }
};


class BoxObject : public BulletObject {
private:
  btScalar mass;
  btVector3 halfExtents;

public:
    typedef boost::shared_ptr<BoxObject> Ptr;

    BoxObject(btScalar mass_, btVector3 halfExtents_,
                 boost::shared_ptr<btDefaultMotionState> motionState_);
    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new BoxObject(*this));
        internalCopy(o, f);
        return o;
    }
};

// A wrapper for btCapsuleShapeX
class CapsuleObject : public BulletObject {
private: 
    btScalar mass, radius, height;

public:
    typedef boost::shared_ptr<CapsuleObject> Ptr;

    CapsuleObject(btScalar mass_, btScalar radius_, btScalar height_,
                  boost::shared_ptr<btDefaultMotionState> motionState_);
    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new CapsuleObject(*this));
        internalCopy(o, f);
        return o;
    }
    osg::ref_ptr<osg::Node> createOSGNode();
};

#endif // _BASICOBJECTS_H_
