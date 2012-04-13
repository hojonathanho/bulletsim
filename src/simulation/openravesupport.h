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

class RaveRobotObject : public CompoundObject<BulletObject> {
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
    void initRobot(const btTransform &initialTransform, TrimeshMode trimeshMode, float fmargin, bool isDynamic);

    // empty constructor for manual copying
    RaveRobotObject(btScalar scale_) : scale(scale_) { }

public:
    typedef boost::shared_ptr<RaveRobotObject> Ptr;

    RobotBasePtr robot;
    const btScalar scale;

    // this class is actually a collection of BulletObjects,
    // each of which represents a link of the robot
    RaveRobotObject(RaveInstance::Ptr rave_,
            const std::string &uri,
            const btTransform &initialTransform_,
            btScalar scale=1.0f,
            TrimeshMode trimeshMode=CONVEX_HULL,
            bool isDynamic = false
            );

    void setTransform(const btTransform &t) { initialTransform = t; updateBullet(); }
    btTransform getTransform() const { return initialTransform; }
    btTransform toRobotFrame(const btTransform &t) const { return util::scaleTransform(initialTransform.inverse() * t, 1./scale); }
    btTransform toWorldFrame(const btTransform &t) const { return initialTransform * util::scaleTransform(t, scale); }

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
        return toWorldFrame(util::toBtTransform(link->GetTransform()));
    }

    void ignoreCollisionWith(const btCollisionObject *obj) { ignoreCollisionObjs.insert(obj); }
    // Returns true if the robot's current pose collides with anything in the environment
    // (this will call updateAabbs() on the dynamicsWorld)
    bool detectCollisions();

    // Positions the robot according to DOF values in the OpenRAVE model
    // and copy link positions to the Bullet rigid bodies.
    void setDOFValues(const vector<int> &indices, const vector<dReal> &vals);
    void updateBullet();

    vector<double> getDOFValues(const vector<int> &indices);

    // IK support
    struct Manipulator {
        RaveRobotObject *robot;
        ModuleBasePtr ikmodule;
        RobotBase::ManipulatorPtr manip, origManip;
        int index; // id for this manipulator in this robot instance

        bool useFakeGrabber;
        GrabberKinematicObject::Ptr grabber;
        void updateGrabberPos();

        typedef boost::shared_ptr<Manipulator> Ptr;
        Manipulator(RaveRobotObject *robot_) : robot(robot_) { }

        btTransform getTransform() const {
            return robot->toWorldFrame(util::toBtTransform(manip->GetTransform()));
        }
        vector<double> getDOFValues();
        float getGripperAngle();
        void setGripperAngle(float);

        // Gets one IK solution closest to the current position in joint space
        bool solveIKUnscaled(const OpenRAVE::Transform &targetTrans,
                vector<dReal> &vsolution);
        bool solveIK(const btTransform &targetTrans, vector<dReal> &vsolution) {
            return solveIKUnscaled(
                    util::toRaveTransform(robot->toRobotFrame(targetTrans)),
                    vsolution);
        }

        // Gets all IK solutions
        bool solveAllIKUnscaled(const OpenRAVE::Transform &targetTrans,
                vector<vector<dReal> > &vsolutions);
        bool solveAllIK(const btTransform &targetTrans,
                vector<vector<dReal> > &vsolutions) {
            return solveAllIKUnscaled(
                    util::toRaveTransform(robot->toRobotFrame(targetTrans)),
                    vsolutions);
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
            return moveByIKUnscaled(util::toRaveTransform(robot->toRobotFrame(targetTrans)),
                    checkCollisions, revertOnCollision);
        }

        Manipulator::Ptr copy(RaveRobotObject::Ptr newRobot, Fork &f);
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
