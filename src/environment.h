#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_

#include <osg/MatrixTransform>
#include <btBulletDynamicsCommon.h>
#include <list>
#include <boost/shared_ptr.hpp>

struct OSGInstance {
    typedef boost::shared_ptr<OSGInstance> Ptr;

    osg::ref_ptr<osg::Group> root;

    OSGInstance();
};

struct BulletInstance {
    typedef boost::shared_ptr<BulletInstance> Ptr;

    btBroadphaseInterface *broadphase;
    btDefaultCollisionConfiguration *collisionConfiguration;
    btCollisionDispatcher *dispatcher;
    btSequentialImpulseConstraintSolver *solver;
    btDiscreteDynamicsWorld *dynamicsWorld;

    BulletInstance();
    ~BulletInstance();
};

class Environment;
class EnvironmentObject {
private:
    Environment *env;

public:
    typedef boost::shared_ptr<EnvironmentObject> Ptr;

    EnvironmentObject() { }
    virtual ~EnvironmentObject() { }

    Environment *getEnvironment() { return env; }

    // methods only to be called by the Environment
    void setEnvironment(Environment *env_) { env = env_; }
    virtual void init() { }
    virtual void prePhysics() { }
    virtual void preDraw() { }
};

struct Environment {
    typedef boost::shared_ptr<Environment> Ptr;

    BulletInstance::Ptr bullet;
    OSGInstance::Ptr osg;

    typedef std::list<EnvironmentObject::Ptr> ObjectList;
    ObjectList objects;

    Environment(BulletInstance::Ptr bullet_, OSGInstance::Ptr osg_) : bullet(bullet_), osg(osg_) { }

    void add(EnvironmentObject::Ptr obj);
    void step(btScalar dt);
};


// objects

// Not a real object; just wraps a bunch of child objects.
template<class ChildType>
class CompoundObject : public EnvironmentObject {
protected:
    std::vector<ChildType> children;
    std::vector<ChildType> &getChildren() { return children; }

public:
    void init() {
        typename std::vector<ChildType>::iterator i;
        for (i = children.begin(); i != children.end(); ++i) {
            if (*i) {
                (*i)->setEnvironment(getEnvironment());
                (*i)->init();
            }
        }
    }

    void prePhysics() {
        typename std::vector<ChildType>::iterator i;
        for (i = children.begin(); i != children.end(); ++i)
            if (*i)
                (*i)->prePhysics();
    }

    void preDraw() {
        typename std::vector<ChildType>::iterator i;
        for (i = children.begin(); i != children.end(); ++i)
            if (*i)
                (*i)->preDraw();
    }
};

// an object that is entirely specified as a bullet btRigidBody
// (the OSG model will be created from the btRigidBody, and
// will be added to the scene graph and the dynamics world, respectively)
class BulletObject : public EnvironmentObject {
public:
    typedef boost::shared_ptr<EnvironmentObject> Ptr;

    boost::shared_ptr<btRigidBody> rigidBody;
    boost::shared_ptr<btMotionState> motionState;
    boost::shared_ptr<btCollisionShape> collisionShape;

    osg::ref_ptr<osg::Node> node;
    osg::ref_ptr<osg::MatrixTransform> transform;

    BulletObject() { }
    BulletObject(boost::shared_ptr<btCollisionShape> collisionShape_, boost::shared_ptr<btMotionState> motionState_,
                 boost::shared_ptr<btRigidBody> rigidBody_) :
        collisionShape(collisionShape_), motionState(motionState_), rigidBody(rigidBody_) { }
    virtual ~BulletObject() { }

    // called by Environment
    void init();
    void preDraw();

    // methods to be overridden
    virtual void initBulletStructures() { }; // should set rigidBody, motionState, collisionShape, if not set by constructor
    // by default uses osgBullet. Can be overridden to provide custom OSG mesh
    virtual osg::ref_ptr<osg::Node> createOSGNode();
};

class BulletKinematicObject : public BulletObject {
public:
    typedef boost::shared_ptr<BulletKinematicObject> Ptr;

    // this is a motion state for kinematic objects, as described at
    // http://bulletphysics.org/mediawiki-1.5.8/index.php/MotionStates
    struct MotionState : public btMotionState {
        typedef boost::shared_ptr<MotionState> Ptr;
        btTransform trans;
        MotionState(const btTransform &trans_) : trans(trans_) { }
        void getWorldTransform(btTransform &worldTrans) const { worldTrans = trans; }
        void setWorldTransform(const btTransform &) { }
        void setKinematicPos(const btTransform &pos) { trans = pos; }
    };

    BulletKinematicObject(boost::shared_ptr<btCollisionShape> collisionShape_, const btTransform &trans);
    MotionState &getKinematicMotionState() { return *static_cast<MotionState *> (motionState.get()); }
};

#endif // _ENVIRONMENT_H_
