#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_

#include <osg/MatrixTransform>
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <iostream>
using namespace std;

struct OSGInstance {
    typedef boost::shared_ptr<OSGInstance> Ptr;

    osg::ref_ptr<osg::Group> root;

    OSGInstance();
};

struct BulletInstance {
    typedef boost::shared_ptr<BulletInstance> Ptr;

    btBroadphaseInterface *broadphase;
    btSoftBodyRigidBodyCollisionConfiguration *collisionConfiguration;
    btCollisionDispatcher *dispatcher;
    btSequentialImpulseConstraintSolver *solver;
    btSoftRigidDynamicsWorld *dynamicsWorld;
    btSoftBodyWorldInfo softBodyWorldInfo;

    BulletInstance();
    ~BulletInstance();

    void setGravity(const btVector3 &gravity);
    void reset();
};

class Environment;
class EnvironmentObject {
private:
    Environment *env;

public:
    typedef boost::shared_ptr<EnvironmentObject> Ptr;

    EnvironmentObject() { }
    EnvironmentObject(Environment *env_) : env(env_) { }
    virtual ~EnvironmentObject() { }

    Environment *getEnvironment() { return env; }

    virtual EnvironmentObject::Ptr copy() = 0;

    // methods only to be called by the Environment
    void setEnvironment(Environment *env_) { env = env_; }
    virtual void init() { }
    virtual void prePhysics() { }
    virtual void preDraw() { }
    virtual void destroy() { }
};

struct Environment {
    typedef boost::shared_ptr<Environment> Ptr;

    BulletInstance::Ptr bullet;
    OSGInstance::Ptr osg;

    typedef std::vector<EnvironmentObject::Ptr> ObjectList;
    ObjectList objects;

    Environment(BulletInstance::Ptr bullet_, OSGInstance::Ptr osg_) : bullet(bullet_), osg(osg_) { }
    ~Environment();

    void add(EnvironmentObject::Ptr obj);
    void step(btScalar dt, int maxSubSteps, btScalar fixedTimeStep);

    // An Environment::Fork is a wrapper around an Environment with an operator
    // that associates copied objects with their original ones
    struct Fork {
        typedef boost::shared_ptr<Fork> Ptr;

        Environment *parentEnv;
        Environment::Ptr env;

        typedef std::map<EnvironmentObject::Ptr, EnvironmentObject::Ptr> ObjectMap;
        ObjectMap objMap; // maps object in parentEnv to object in env

        Fork(Environment *parentEnv_, BulletInstance::Ptr bullet, OSGInstance::Ptr osg);

        EnvironmentObject::Ptr forkOf(EnvironmentObject::Ptr orig);
    };
    Fork::Ptr fork(BulletInstance::Ptr newBullet, OSGInstance::Ptr newOSG);
};


// objects

// Not a real object; just wraps a bunch of child objects.
template<class ChildType>
class CompoundObject : public EnvironmentObject {
protected:
    typedef std::vector<typename ChildType::Ptr> ChildVector;

public:
    typedef boost::shared_ptr<CompoundObject<ChildType> > Ptr;
    ChildVector children;
    ChildVector &getChildren() { return children; }

    CompoundObject() { }
    // copy constructor
    CompoundObject(const CompoundObject<ChildType> &c) {
        children.reserve(c.children.size());
        typename ChildVector::const_iterator i;
        for (i = c.children.begin(); i != c.children.end(); ++i)
            children.push_back(boost::static_pointer_cast<ChildType> ((*i)->copy()));
    }

    EnvironmentObject::Ptr copy() { return Ptr(new CompoundObject(*this)); }

    void init() {
        typename ChildVector::iterator i;
        for (i = children.begin(); i != children.end(); ++i) {
            if (*i) {
                (*i)->setEnvironment(getEnvironment());
                (*i)->init();
            }
        }
    }

    void prePhysics() {
        typename ChildVector::iterator i;
        for (i = children.begin(); i != children.end(); ++i)
            if (*i)
                (*i)->prePhysics();
    }

    void preDraw() {
        typename ChildVector::iterator i;
        for (i = children.begin(); i != children.end(); ++i)
            if (*i)
                (*i)->preDraw();
    }

    void destroy() {
        typename ChildVector::iterator i;
        for (i = children.begin(); i != children.end(); ++i)
            if (*i)
                (*i)->destroy();
    }
};

// an object that is entirely specified as a bullet btRigidBody
// (the OSG model will be created from the btRigidBody, and
// will be added to the scene graph and the dynamics world, respectively)
class BulletObject : public EnvironmentObject {
public:
    typedef boost::shared_ptr<BulletObject> Ptr;

    boost::shared_ptr<btRigidBody> rigidBody;
    // the motionState and collisionShape actually don't matter; the ones
    // embedded in the rigidBody are used for simulation. However,
    // placing them here will have them automatically deallocated
    // on destruction of the BulletObject
    boost::shared_ptr<btDefaultMotionState> motionState;
    boost::shared_ptr<btCollisionShape> collisionShape;

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

    EnvironmentObject::Ptr copy() { return Ptr(new BulletObject(*this)); }

    // called by Environment
    void init();
    void preDraw();
    void destroy();

    // methods to be overridden
    virtual void initBulletStructures() { }; // should set rigidBody, motionState, collisionShape, if not set by constructor
    // by default uses osgBullet. Can be overridden to provide custom OSG mesh
    virtual osg::ref_ptr<osg::Node> createOSGNode();
  void setColor(float r, float g, float b, float a);
};

class BulletKinematicObject : public BulletObject {
public:
    typedef boost::shared_ptr<BulletKinematicObject> Ptr;

    // this is a motion state for kinematic objects, as described at
    // http://bulletphysics.org/mediawiki-1.5.8/index.php/MotionStates
    struct MotionState : public btDefaultMotionState {
        typedef boost::shared_ptr<MotionState> Ptr;
        MotionState(const btTransform &trans) : btDefaultMotionState(trans) { }
        void setWorldTransform(const btTransform &) { }
        void setKinematicPos(const btTransform &pos) { btDefaultMotionState::setWorldTransform(pos); }
    };

    BulletKinematicObject(boost::shared_ptr<btCollisionShape> collisionShape_, const btTransform &trans);
    EnvironmentObject::Ptr copy() { return Ptr(new BulletKinematicObject(*this)); }

    MotionState &getKinematicMotionState() { return *static_cast<MotionState *> (motionState.get()); }
};

#endif // _ENVIRONMENT_H_
