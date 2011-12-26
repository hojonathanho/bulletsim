#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_

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

    // Populates out with all objects colliding with obj, possibly ignoring some objects
    // dynamicsWorld->updateAabbs() must be called before contactTest
    // see http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?t=4850
    typedef std::set<const btCollisionObject *> CollisionObjectSet;
    void contactTest(btCollisionObject *obj, CollisionObjectSet &out, const CollisionObjectSet *ignore=NULL);
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
    ChildVector children;
    ChildVector &getChildren() { return children; }

public:
    typedef boost::shared_ptr<CompoundObject<ChildType> > Ptr;

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

class Action {
protected:
    bool isDone;
    void setDone(bool b) { isDone = b; }

public:
    typedef boost::shared_ptr<Action> Ptr;
    Action() : isDone(false) { }
    bool done() const { return isDone; }
    virtual void step(float dt) = 0;
};

#endif // _ENVIRONMENT_H_
