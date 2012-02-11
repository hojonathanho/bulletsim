#ifndef _OPENRAVESUPPORT_H_
#define _OPENRAVESUPPORT_H_

#include <openrave/openrave.h>
using namespace OpenRAVE;

#include <btBulletDynamicsCommon.h>

#include <vector>
using namespace std;

#include "environment.h"
#include "basicobjects.h"
#include "util.h"

struct RaveInstance {
    typedef boost::shared_ptr<RaveInstance> Ptr;

    bool isRoot;
    EnvironmentBasePtr env;

    RaveInstance();
    // copy constructor. will never call RaveInitialize or RaveDestroy
    RaveInstance(const RaveInstance &o, int cloneOpts);
    ~RaveInstance();
};

enum TrimeshMode {
    CONVEX_DECOMP, // use HACD convex decomposition
    CONVEX_HULL, // use btShapeHull
    RAW // use btBvhTriangleMeshShape (not recommended, makes simulation very slow)
};

class RaveRobotKinematicObject : public CompoundObject<BulletObject> {
private:
    RaveInstance::Ptr rave;
    btTransform initialTransform;

    // these two containers just keep track of the smart pointers
    // so that the objects get deallocated on destruction
    std::vector<boost::shared_ptr<btStridingMeshInterface> > meshes;
    std::vector<boost::shared_ptr<btCollisionShape> > subshapes;

    // for looking up the associated Bullet object for an OpenRAVE link
    std::map<KinBody::LinkPtr, BulletObject::Ptr> linkMap;
    std::map<btCollisionObject *, KinBody::LinkPtr> collisionObjMap;

    // maps a child to a position in the children array. used for copying
    std::map<BulletObject::Ptr, int> childPosMap;

    // vector of objects to ignore collision with
    BulletInstance::CollisionObjectSet ignoreCollisionObjs;

    // for the loaded robot, this will create BulletObjects
    // and place them into the children vector
    void initRobotWithoutDynamics(const btTransform &initialTransform, TrimeshMode trimeshMode, float fmargin=0.0005);

    // empty constructor for manual copying
    RaveRobotKinematicObject(btScalar scale_) : scale(scale_) { }

public:
    typedef boost::shared_ptr<RaveRobotKinematicObject> Ptr;

    RobotBasePtr robot;
    const btScalar scale;

    // this class is actually a collection of BulletObjects,
    // each of which represents a link of the robot
    RaveRobotKinematicObject(RaveInstance::Ptr rave_,
            const std::string &uri,
            const btTransform &initialTransform_,
            btScalar scale=1.0f,
            TrimeshMode trimeshMode=CONVEX_HULL
            );

    // forking
    EnvironmentObject::Ptr copy(Fork &f) const;
    void postCopy(EnvironmentObject::Ptr copy, Fork &f) const;

    // Gets equivalent rigid bodies in OpenRAVE and in Bullet
    BulletObject::Ptr associatedObj(KinBody::LinkPtr link) const {
        std::map<KinBody::LinkPtr, BulletObject::Ptr>::const_iterator i = linkMap.find(link);
        return i == linkMap.end() ? BulletObject::Ptr() : i->second;
    }
    KinBody::LinkPtr associatedObj(btCollisionObject *obj) const {
        std::map<btCollisionObject *, KinBody::LinkPtr>::const_iterator i = collisionObjMap.find(obj);
        return i == collisionObjMap.end() ? KinBody::LinkPtr() : i->second;
    }

    // When getting transforms of links, you must remember to scale!
    // or just get the transforms directly from the equivalent Bullet rigid bodies
    btTransform getLinkTransform(KinBody::LinkPtr link) const {
        return util::toBtTransform(link->GetTransform(), scale);
    }

    void ignoreCollisionWith(const btCollisionObject *obj) { ignoreCollisionObjs.insert(obj); }
    // Returns true if the robot's current pose collides with anything in the environment
    // (this will call updateAabbs() on the dynamicsWorld)
    bool detectCollisions();

    // Positions the robot according to DOF values in the OpenRAVE model
    // and copy link positions to the Bullet rigid bodies.
    void setDOFValues(const vector<int> &indices, const vector<dReal> &vals);

    // IK support
    struct Manipulator {
        RaveRobotKinematicObject *robot;
        ModuleBasePtr ikmodule;
        RobotBase::ManipulatorPtr manip;
        int index; // id for this manipulator in this robot instance

        bool useFakeGrabber;
        GrabberKinematicObject::Ptr grabber;
        void updateGrabberPos();

        typedef boost::shared_ptr<Manipulator> Ptr;
        Manipulator(RaveRobotKinematicObject *robot_) : robot(robot_) { }

        btTransform getTransform() const {
            return util::toBtTransform(manip->GetTransform(), robot->scale);
        }

        // Moves the manipulator with IK to targetTrans in unscaled coordinates
        // Returns false if IK cannot find a solution
        // If checkCollisions is true, then this will return false if the new
        // robot pose collides with anything in the environment (true otherwise).
        // The robot will revert is position to the pre-collision state if
        // revertOnCollision is set to true.
        bool moveByIKUnscaled(const OpenRAVE::Transform &targetTrans,
                bool checkCollisions=false, bool revertOnCollision=true);
        // Moves the manipulator in scaled coordinates
        bool moveByIK(const btTransform &targetTrans,
                bool checkCollisions=false, bool revertOnCollision=true) {
            return moveByIKUnscaled(util::toRaveTransform(targetTrans, 1./robot->scale),
                checkCollisions, revertOnCollision);
        }

        Manipulator::Ptr copy(RaveRobotKinematicObject::Ptr newRobot, Fork &f);
    };

    // If useFakeGrabber is true, the manipulator will use a GrabberKinematicObject
    // which can "grab" objects by simply setting a point constraint with the nearest
    // object in front of the manipulator. Pass in false for realistic grasping.
    Manipulator::Ptr createManipulator(const std::string &manipName, bool useFakeGrabber=false);
    void destroyManipulator(Manipulator::Ptr m); // not necessary to call this on destruction
    Manipulator::Ptr getManipByIndex(int i) const { return createdManips[i]; }
    int numCreatedManips() const { return createdManips.size(); }
private:
    std::vector<Manipulator::Ptr> createdManips;
};

#endif // _OPENRAVESUPPORT_H_
