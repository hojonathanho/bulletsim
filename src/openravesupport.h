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

class RaveRobotKinematicObject : public CompoundObject<BulletKinematicObject::Ptr> {
private:
    RaveInstance::Ptr rave;
    btTransform initialTransform;

    // these two containers just keep track of the smart pointers
    // so that the objects get deallocated on destruction
    std::vector<boost::shared_ptr<btStridingMeshInterface> > meshes;
    std::vector<boost::shared_ptr<btCollisionShape> > subshapes;

    // for the loaded robot, this will create BulletKinematicObjects
    // and place them into the children vector
    void initRobotWithoutDynamics(const btTransform &initialTransform, float fmargin=0.0005);

public:
    typedef boost::shared_ptr<RaveRobotKinematicObject> Ptr;

    RobotBasePtr robot;
    const btScalar scale;

    // this class is actually a collection of BulletKinematicObjects,
    // each of which represents a link of the robot
    RaveRobotKinematicObject(RaveInstance::Ptr rave_, const std::string &uri, const btTransform &initialTransform_, btScalar scale=1.0f);

    // position the robot according to DOF values in the OpenRAVE model
    // and copy link positions to the Bullet rigid bodies
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
        void moveByIKUnscaled(const OpenRAVE::Transform &targetTrans); // note: targetTrans is in unscaled coordinates
        void moveByIK(const btTransform &targetTrans);
    };
    Manipulator::Ptr createManipulator(const std::string &manipName);
};

#endif // _OPENRAVESUPPORT_H_
