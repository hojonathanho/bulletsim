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

struct Environment;
struct Fork;
class EnvironmentObject {
private:
    Environment *env;

public:
    typedef boost::shared_ptr<EnvironmentObject> Ptr;

    EnvironmentObject() { }
    EnvironmentObject(Environment *env_) : env(env_) { }
    virtual ~EnvironmentObject() { }

    Environment *getEnvironment() { return env; }

    // These are for environment forking.
    // copy() should return a copy of the object suitable for addition
    // into the environment contained by f. This should NOT add objects to f.env;
    // however it should call f.registerCopy() for each Bullet
    // collision object that it makes.
    virtual EnvironmentObject::Ptr copy(Fork &f) const = 0;
    // postCopy is called after all objects are copied and inserted into f.env.
    // This is useful for updating constraints, anchors, etc.
    // You're free to use f.forkOf()  or f.copyOf() to get equivalent objects in the new env.
    virtual void postCopy(EnvironmentObject::Ptr copy, Fork &f) const { }

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

    typedef std::vector<EnvironmentObject::Ptr> ConstraintList;
    ConstraintList constraints;

    Environment(BulletInstance::Ptr bullet_, OSGInstance::Ptr osg_) : bullet(bullet_), osg(osg_) { }
    ~Environment();

    void add(EnvironmentObject::Ptr obj);
    void remove(EnvironmentObject::Ptr obj);

    void addConstraint(EnvironmentObject::Ptr cnt);
    void removeConstraint(EnvironmentObject::Ptr cnt);

    void step(btScalar dt, int maxSubSteps, btScalar fixedTimeStep);
};


// An Environment Fork is a wrapper around an Environment with an operator
// that associates copied objects with their original ones
class Fork {
    void copyObjects();

public:
    typedef boost::shared_ptr<Fork> Ptr;

    const Environment *parentEnv;
    Environment::Ptr env;

    typedef std::map<EnvironmentObject::Ptr, EnvironmentObject::Ptr> ObjectMap;
    ObjectMap objMap; // maps object in parentEnv to object in env

    typedef std::map<const void *, void *> DataMap;
    DataMap dataMap;
    void registerCopy(const void *orig, void *copy) {
        BOOST_ASSERT(copyOf(orig) == NULL && orig && copy);
        dataMap.insert(std::make_pair(orig, copy));
    }

    Fork(const Environment *parentEnv_, BulletInstance::Ptr bullet, OSGInstance::Ptr osg) :
        parentEnv(parentEnv_), env(new Environment(bullet, osg)) { copyObjects(); }
    Fork(const Environment::Ptr parentEnv_, BulletInstance::Ptr bullet, OSGInstance::Ptr osg) :
        parentEnv(parentEnv_.get()), env(new Environment(bullet, osg)) { copyObjects(); }

    void *copyOf(const void *orig) const {
        DataMap::const_iterator i = dataMap.find(orig);
        return i == dataMap.end() ? NULL : i->second;
    }
    EnvironmentObject::Ptr forkOf(EnvironmentObject::Ptr orig) const {
        ObjectMap::const_iterator i = objMap.find(orig);
        return i == objMap.end() ? EnvironmentObject::Ptr() : i->second;
    }

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

    EnvironmentObject::Ptr copy(Fork &f) const {
        Ptr o(new CompoundObject<ChildType>());
        internalCopy(o, f);
        return o;
    }

    void internalCopy(CompoundObject::Ptr o, Fork &f) const {
        o->children.reserve(children.size());
        typename ChildVector::const_iterator i;
        for (i = children.begin(); i != children.end(); ++i) {
            if (*i)
                o->children.push_back(boost::static_pointer_cast<ChildType> ((*i)->copy(f)));
            else
                o->children.push_back(typename ChildType::Ptr());
        }
    }

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
    float timeElapsed;
    float execTime;
    bool isDone;
    int plotOnly;

    void setDone(bool b) { isDone = b; }
    void stepTime(float dt) { timeElapsed += dt; }
    float fracElapsed() const { return min(timeElapsed / execTime, 1.f); }
    void setColor(float r, float g, float b, float a);

public:
    typedef boost::shared_ptr<Action> Ptr;
    Action() : isDone(false), timeElapsed(0.), execTime(1.) { }
    Action(float execTime_) : isDone(false), timeElapsed(0.), execTime(execTime_) { }

    bool done() const { return timeElapsed >= execTime || isDone; }
    virtual void step(float dt) = 0;
    virtual void reset() { timeElapsed = 0.; setDone(false); }
    virtual void setExecTime(float t) { execTime = t; }
};

#endif // _ENVIRONMENT_H_
