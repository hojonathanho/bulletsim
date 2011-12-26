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

    EnvironmentBasePtr env;

    RaveInstance();
    ~RaveInstance();
};

class RaveRobotKinematicObject : public CompoundObject<BulletKinematicObject> {
private:
    RaveInstance::Ptr rave;
    btTransform initialTransform;

    // these two containers just keep track of the smart pointers
    // so that the objects get deallocated on destruction
    std::vector<boost::shared_ptr<btStridingMeshInterface> > meshes;
    std::vector<boost::shared_ptr<btCollisionShape> > subshapes;

    // vector of objects to ignore collision with
    BulletInstance::CollisionObjectSet ignoreCollisionObjs;

    // for the loaded robot, this will create BulletKinematicObjects
    // and place them into the children vector
    void initRobotWithoutDynamics(const btTransform &initialTransform, bool useConvexHull=true, float fmargin=0.0005);

public:
    typedef boost::shared_ptr<RaveRobotKinematicObject> Ptr;

    RobotBasePtr robot;
    const btScalar scale;

    // this class is actually a collection of BulletKinematicObjects,
    // each of which represents a link of the robot
    RaveRobotKinematicObject(RaveInstance::Ptr rave_, const std::string &uri, const btTransform &initialTransform_, btScalar scale=1.0f);

    void ignoreCollisionWith(const btCollisionObject *obj) { ignoreCollisionObjs.insert(obj); }
    // Returns true if the robot's current pose collides with anything in the environment
    // It's the caller's responsibility to either call BulletInstance::detectCollisions()
    // or dynamicsWorld->stepSimulation() beforehand.
    bool detectCollisions();

    // Positions the robot according to DOF values in the OpenRAVE model
    // and copy link positions to the Bullet rigid bodies.
    void setDOFValues(const vector<int> &indices, const vector<dReal> &vals);

    // IK support
    struct Manipulator {
        RaveRobotKinematicObject *robot;
        ModuleBasePtr ikmodule;
        RobotBase::ManipulatorPtr manip;
        GrabberKinematicObject::Ptr grabber;
        void updateGrabberPos();

        typedef boost::shared_ptr<Manipulator> Ptr;
        Manipulator(RaveRobotKinematicObject *robot_) : robot(robot_) { }

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
    };
    Manipulator::Ptr createManipulator(const std::string &manipName);
};

#endif // _OPENRAVESUPPORT_H_
