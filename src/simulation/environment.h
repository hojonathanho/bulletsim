#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <vector>
#include <set>
#include <map>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <stdexcept>

using namespace std;

struct BulletInstance {
    typedef boost::shared_ptr<BulletInstance> Ptr;

    btBroadphaseInterface *broadphase;
    btSoftBodyRigidBodyCollisionConfiguration *collisionConfiguration;
    btCollisionDispatcher *dispatcher;
    btSequentialImpulseConstraintSolver *solver;
    btSoftRigidDynamicsWorld *dynamicsWorld;
    btSoftBodyWorldInfo *softBodyWorldInfo;

    BulletInstance();
    ~BulletInstance();

    void setGravity(const btVector3 &gravity);
    void setDefaultGravity();

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
    virtual void destroy() { }

		//gets the index of the closest part of the object (face, capsule, rigid_body, etc)
		//for rigid bodies, there is only one index so this will always return 0
		virtual int getIndex(const btTransform& transform) { throw std::runtime_error("getIndex() hasn't been defined yet"); return 0;}
		virtual int getIndexSize() { std::runtime_error("getIndex() hasn't been defined yet"); return 0;}
		//gets the transform of the indexed part
		//for rigid bodies, this just returns the rigid body's transform
		virtual btTransform getIndexTransform(int index) { std::runtime_error("getIndexTransform() hasn't been defined yet"); return btTransform();}
};

class RaveInstance;
typedef boost::shared_ptr<RaveInstance> RaveInstancePtr;
struct Environment {
    typedef boost::shared_ptr<Environment> Ptr;

    BulletInstance::Ptr bullet;

    typedef std::vector<EnvironmentObject::Ptr> ObjectList;
    ObjectList objects;

    typedef std::vector<EnvironmentObject::Ptr> ConstraintList;
    ConstraintList constraints;

    Environment(BulletInstance::Ptr bullet_) : bullet(bullet_) { }
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
    RaveInstancePtr rave;

    typedef std::map<EnvironmentObject *, EnvironmentObject::Ptr> ObjectMap;
    ObjectMap objMap; // maps object in parentEnv to object in env

    typedef std::map<const void *, void *> DataMap;
    DataMap dataMap;
    void registerCopy(const void *orig, void *copy) {
        BOOST_ASSERT(copyOf(orig) == NULL && orig && copy);
        dataMap.insert(std::make_pair(orig, copy));
    }

    Fork(const Environment *parentEnv_, BulletInstance::Ptr bullet);
    Fork(const Environment::Ptr parentEnv_, BulletInstance::Ptr bullet);
    Fork(const Environment::Ptr parentEnv_, const RaveInstancePtr rave_, BulletInstance::Ptr bullet);

    void *copyOf(const void *orig) const {
        DataMap::const_iterator i = dataMap.find(orig);
        return i == dataMap.end() ? NULL : i->second;
    }
    EnvironmentObject::Ptr forkOf(EnvironmentObject::Ptr orig) const {
        ObjectMap::const_iterator i = objMap.find(orig.get());
        return i == objMap.end() ? EnvironmentObject::Ptr() : i->second;
    }
    EnvironmentObject::Ptr forkOf(EnvironmentObject *orig) const {
        ObjectMap::const_iterator i = objMap.find(orig);
        return i == objMap.end() ? EnvironmentObject::Ptr() : i->second;
    }

};

// objects

// Not a real object; just wraps a bunch of child objects.
template<class ChildType>
class CompoundObject : public EnvironmentObject {
public:
    typedef boost::shared_ptr<CompoundObject<ChildType> > Ptr;

    typedef std::vector<typename ChildType::Ptr> ChildVector;
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

    virtual void init() {
        typename ChildVector::iterator i;
        for (i = children.begin(); i != children.end(); ++i) {
            if (*i) {
                (*i)->setEnvironment(getEnvironment());
                (*i)->init();
            }
        }
    }

    virtual void prePhysics() {
        typename ChildVector::iterator i;
        for (i = children.begin(); i != children.end(); ++i)
            if (*i)
                (*i)->prePhysics();
    }

    virtual void destroy() {
        typename ChildVector::iterator i;
        for (i = children.begin(); i != children.end(); ++i)
            if (*i)
                (*i)->destroy();
    }

		int getIndex(const btTransform& transform) {
			const btVector3 pos = transform.getOrigin();
			int j_nearest = -1;
			float nearest_length2 = DBL_MAX;
			for (int i=0; i<children.size(); i++) {
				int j = i*children[i]->getIndexSize() + children[i]->getIndex(transform);
				const btVector3 center = children[i]->getIndexTransform(j % children[i]->getIndexSize()).getOrigin();
				const float length2 = (pos - center).length2();
				if (length2 < nearest_length2) {
					j_nearest = j;
					nearest_length2 = length2;
				}
			}
			return j_nearest;
		}

		int getIndexSize() {
			return children.size() * children[0]->getIndexSize();
		}

		btTransform getIndexTransform(int index) {
			const int index_size = children[0]->getIndexSize();
			return children[index/index_size]->getIndexTransform(index % index_size);
		}

};

#endif // _ENVIRONMENT_H_
