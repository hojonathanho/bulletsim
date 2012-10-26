#ifndef _BASICOBJECTS_H_
#define _BASICOBJECTS_H_

#include <opencv2/core/core.hpp>
#include <osg/MatrixTransform>
#include "environment.h"

// an object that is entirely specified as a bullet btRigidBody
// (the OSG model will be created from the btRigidBody, and
// will be added to the scene graph and the dynamics world, respectively)
class BulletObject : public EnvironmentObject {
public:
    struct MotionState : public btDefaultMotionState {
        typedef boost::shared_ptr<MotionState> Ptr;
        BulletObject &obj;

        MotionState(BulletObject &obj_, const btTransform &trans) :
            obj(obj_), btDefaultMotionState(trans) { }

        void setWorldTransform(const btTransform &t) {
            if (!obj.isKinematic)
                btDefaultMotionState::setWorldTransform(t);
        }

        void setKinematicPos(const btTransform &pos) {
						if (!obj.isKinematic) cout << "warning! called setKinematicPos on non-kinematic object." << endl;;
            btDefaultMotionState::setWorldTransform(pos);
            // if we want to do collision detection in between timesteps,
            // we also have to directly set this
            obj.rigidBody->setCenterOfMassTransform(pos);
        }

        Ptr clone(BulletObject &newObj);
    };


    typedef boost::shared_ptr<BulletObject> Ptr;

    // BULLET MEMBERS
    boost::shared_ptr<btRigidBody> rigidBody;
    // the motionState and collisionShape actually don't matter; the ones
    // embedded in the rigidBody are used for simulation. However,
    // placing them here will have them automatically deallocated
    // on destruction of the BulletObject
    MotionState::Ptr motionState;
    boost::shared_ptr<btCollisionShape> collisionShape;
    boost::shared_ptr<btCollisionShape> graphicsShape;

    // OSG MEMBERS
    osg::ref_ptr<osg::Node> node;
    osg::ref_ptr<osg::MatrixTransform> transform;

    // CONSTRUCTORS
    // our own construction info class, which forces motionstate to be null
    struct CI : public btRigidBody::btRigidBodyConstructionInfo {
        CI(btScalar mass, btCollisionShape *collisionShape, const btVector3 &localInertia=btVector3(0,0,0)) :
            btRigidBody::btRigidBodyConstructionInfo(mass, NULL, collisionShape, localInertia) { }
    };
    // this constructor computes a ConstructionInfo for you
    BulletObject(btScalar mass, btCollisionShape *cs, const btTransform &initTrans, bool isKinematic_=false);
    BulletObject(btScalar mass, boost::shared_ptr<btCollisionShape> cs, const btTransform &initTrans, bool isKinematic_=false);

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
        btTransform start, end;

    public:
        typedef boost::shared_ptr<MoveAction> Ptr;
        MoveAction(BulletObject *obj_) : obj(obj_) { }
        MoveAction(BulletObject *obj_, const btTransform &start_, const btTransform &end_, float execTime) : obj(obj_), start(start_), end(end_), Action(execTime) { }
        void setEndpoints(const btTransform &start_, const btTransform &end_) {
            start = start_; end = end_;
        }
        void step(float dt);
    };
    MoveAction::Ptr createMoveAction() { return MoveAction::Ptr(new MoveAction(this)); }
    MoveAction::Ptr createMoveAction(const btTransform &start, const btTransform &end, float time) { return MoveAction::Ptr(new MoveAction(this, start, end, time)); }
		void setColor(float r, float g, float b, float a);
		
		void setTexture(const cv::Mat& image);
		void adjustTransparency(float increment);

		int getIndex(const btTransform& transform) { return 0; }
		int getIndexSize() { return 1; }
		btTransform getIndexTransform(int index) { return rigidBody->getCenterOfMassTransform(); }

		void setKinematic(bool);
		bool isKinematic;

private:
		bool enable_texture;
		osg::Vec4f m_color;
		void setColorAfterInit();
		osg::ref_ptr<osg::Image> m_image;
		boost::shared_ptr<cv::Mat> m_cvimage;
		void setTextureAfterInit();
    void setFlagsAndActivation();
    void construct(btScalar mass, boost::shared_ptr<btCollisionShape> cs, const btTransform& initTrans, bool isKinematic_);
public:
		cv::Mat& getTexture() { return *m_cvimage; }
};

class BulletConstraint : public EnvironmentObject {
private:
    BulletConstraint(const BulletConstraint &o);

public:
    typedef boost::shared_ptr<BulletConstraint> Ptr;

    boost::shared_ptr<btTypedConstraint> cnt;
    bool disableCollisionsBetweenLinkedBodies;

    BulletConstraint(btTypedConstraint *cnt_, bool disableCollisionsBetweenLinkedBodies_=false) :
        cnt(cnt_),
        disableCollisionsBetweenLinkedBodies(disableCollisionsBetweenLinkedBodies_) { }
    BulletConstraint(boost::shared_ptr<btTypedConstraint> cnt_, bool disableCollisionsBetweenLinkedBodies_=false) :
        cnt(cnt_),
        disableCollisionsBetweenLinkedBodies(disableCollisionsBetweenLinkedBodies_) { }
    EnvironmentObject::Ptr copy(Fork &f) const;
    void init();
    void destroy();
};

class GrabberKinematicObject : public BulletObject {
private:
    float radius, height;
    btVector3 constraintPivot;
    BulletConstraint::Ptr constraint;

public:
    typedef boost::shared_ptr<GrabberKinematicObject> Ptr;

    GrabberKinematicObject(float radius_, float height_);
    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new GrabberKinematicObject(*this));
        internalCopy(o, f);
        return o;
    }
    void destroy() { releaseConstraint(); BulletObject::destroy(); }

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

    PlaneStaticObject(const btVector3 &planeNormal_, btScalar planeConstant_, const btTransform &initTrans, btScalar drawHalfExtents_=50.);
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

    CylinderStaticObject(btScalar mass_, btScalar radius_, btScalar height_, const btTransform &initTrans);
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

    SphereObject(btScalar mass_, btScalar radius_, const btTransform &initTrans, bool isKinematic=false);
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

    BoxObject(btScalar mass_, const btVector3 &halfExtents_, const btTransform &initTrans);
    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new BoxObject(*this));
        internalCopy(o, f);
        return o;
    }

    inline btVector3 getHalfExtents() { return halfExtents; }
};

// A wrapper for btCapsuleShapeX
class CapsuleObject : public BulletObject {
private: 
    btScalar mass, radius, height;

public:
    typedef boost::shared_ptr<CapsuleObject> Ptr;

    CapsuleObject(btScalar mass_, btScalar radius_, btScalar height_, const btTransform &initTrans);
    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new CapsuleObject(*this));
        internalCopy(o, f);
        return o;
    }
    osg::ref_ptr<osg::Node> createOSGNode();
};

// A wrapper for btCapsuleShape
class CapsuleObjectY : public BulletObject {
private:
    btScalar mass, radius, height;

public:
    typedef boost::shared_ptr<CapsuleObjectY> Ptr;

    CapsuleObjectY(btScalar mass_, btScalar radius_, btScalar height_, const btTransform &initTrans);
    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new CapsuleObjectY(*this));
        internalCopy(o, f);
        return o;
    }
    osg::ref_ptr<osg::Node> createOSGNode();
};

#endif // _BASICOBJECTS_H_
