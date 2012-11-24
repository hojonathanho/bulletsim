#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_

#include <iostream>
#include <stdexcept>
#include <vector>
#include <map>
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <opencv2/core/core.hpp>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

using namespace std;

// callback typedefs
typedef boost::function<bool(const osgGA::GUIEventAdapter &)> Callback;
typedef multimap<osgGA::GUIEventAdapter::EventType, Callback> CallbackMap;
typedef multimap<int, Callback> KeyCallbackMap;
typedef boost::function<void(void)> VoidCallback;
struct VoidCallbackWrapper {
		VoidCallback fn;
		VoidCallbackWrapper(VoidCallback fn_) : fn(fn_) { }
		bool operator()() { fn(); return false; }
};

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
protected:
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

    virtual osg::Node *getOSGNode() const { return NULL; }

    virtual void setColor(float r, float g, float b, float a) {};
		virtual void adjustTransparency(float increment) {};

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
    OSGInstance::Ptr osg;
    boost::shared_ptr<osgbCollision::GLDebugDrawer> dbgDraw;
    bool debugDraw;

    typedef std::vector<EnvironmentObject::Ptr> ObjectList;
    ObjectList objects;

    typedef std::vector<EnvironmentObject::Ptr> ConstraintList;
    ConstraintList constraints;

    Environment();
    ~Environment();

    void add(EnvironmentObject::Ptr obj);
    void remove(EnvironmentObject::Ptr obj);

    void addConstraint(EnvironmentObject::Ptr cnt);
    void removeConstraint(EnvironmentObject::Ptr cnt);

    void step(btScalar dt, int maxSubSteps, btScalar fixedTimeStep);
    void preDraw(); // for drawing without running physics
    void toggleDebugDraw();

    // An Environment can have key callbacks. These will be called if an active Scene contains this Environment.
    KeyCallbackMap keyCallbacks;
    multimap<int, std::string> keyCallbackDescs;
    void addKeyCallback(int c, Callback cb, std::string desc="");
    void addVoidKeyCallback(int c, VoidCallback cb, std::string desc="");
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

    Fork(const Environment *parentEnv_);
    Fork(const Environment::Ptr parentEnv_);
    Fork(const Environment::Ptr parentEnv_, const RaveInstancePtr rave_);

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

    virtual void preDraw() {
        typename ChildVector::iterator i;
        for (i = children.begin(); i != children.end(); ++i)
            if (*i)
                (*i)->preDraw();
    }

    virtual void destroy() {
        typename ChildVector::iterator i;
        for (i = children.begin(); i != children.end(); ++i)
            if (*i)
                (*i)->destroy();
    }

    void setColor(float r, float g, float b, float a) {
			typename ChildVector::iterator i;
			for (i = children.begin(); i != children.end(); ++i)
				if (*i)
					(*i)->setColor(r,g,b,a);
    }

		void setTexture(cv::Mat image) {
			int height = image.size().height;
			int width = image.size().width;
			int n = (int) children.size();
			int split_width = width/n;
			for (int i=0; i<n; i++)
				if (children[i]) {
					cv::Mat splitImage = cv::Mat(image, cv::Rect(i*split_width, 0, split_width, height));
					cv::flip(splitImage, splitImage, 1);
					splitImage = splitImage.t();
					children[i]->setTexture(splitImage);
				}
		}

		void adjustTransparency(float increment) {
			typename ChildVector::iterator i;
      for (i = children.begin(); i != children.end(); ++i)
          if (*i)
              (*i)->adjustTransparency(increment);
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

class ObjectAction {
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
    typedef boost::shared_ptr<ObjectAction> Ptr;
    ObjectAction() : isDone(false), timeElapsed(0.), execTime(1.) { }
    ObjectAction(float execTime_) : isDone(false), timeElapsed(0.), execTime(execTime_) { }

    virtual bool done() const { return timeElapsed >= execTime || isDone; }
    virtual void step(float dt) = 0;
    virtual void reset() { timeElapsed = 0.; setDone(false); }
    virtual void setExecTime(float t) { execTime = t; }
};

#endif // _ENVIRONMENT_H_
