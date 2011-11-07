#ifndef _OPENRAVESUPPORT_H_
#define _OPENRAVESUPPORT_H_

#include <openrave/openrave.h>
using namespace OpenRAVE;

#include <btBulletDynamicsCommon.h>

#include <vector>
using namespace std;

#include "environment.h"
#include "basicobjects.h"

struct RaveInstance {
    typedef boost::shared_ptr<RaveInstance> Ptr;

    EnvironmentBasePtr env;

    RaveInstance();
    ~RaveInstance();
};

class RaveRobotKinematicObject : public EnvironmentObject {
private:
    // utility functions
    static inline btTransform GetBtTransform(const Transform& t) {
        return btTransform(btQuaternion(t.rot.y,t.rot.z,t.rot.w,t.rot.x),GetBtVector(t.trans));
    }

    static inline OpenRAVE::Transform GetRaveTransform(const btTransform &t) {
        OpenRAVE::Transform s;
        s.rot = OpenRAVE::Vector(t.getRotation().w(), t.getRotation().x(), t.getRotation().y(), t.getRotation().z());
        s.trans = OpenRAVE::Vector(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z());
        return s;
    }

    static inline btVector3 GetBtVector(const Vector& v) {
        return btVector3(v.x,v.y,v.z);
    }

    RaveInstance::Ptr rave;
    RobotBasePtr robot;
    btTransform initialTransform;
    std::vector<BulletKinematicObject::Ptr> children;

    // these two containers just keep track of the smart pointers
    // so that the objects get deallocated on destruction
    std::vector<boost::shared_ptr<btStridingMeshInterface> > meshes;
    std::vector<boost::shared_ptr<btCollisionShape> > subshapes;

    // for the loaded robot, this will create BulletKinematicObjects
    // and place them into the children vector
    void initRobotWithoutDynamics(const btTransform &initialTransform, float fmargin=0.0005);

public:
    typedef boost::shared_ptr<RaveRobotKinematicObject> Ptr;

    // this class is actually a collection of BulletKinematicObjects,
    // each of which represents a link of the robot
    RaveRobotKinematicObject(RaveInstance::Ptr rave_, const std::string &uri, const btTransform &initialTransform_);

    // EnvironmentObject methods
    // these act on each sub-object
    void init();
    void prePhysics();
    void preDraw();

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
        void moveByIK(const OpenRAVE::Transform &targetTrans);
        void moveByIK(const btTransform &targetTrans) { moveByIK(GetRaveTransform(targetTrans)); }
    };
    Manipulator::Ptr createManipulator(const std::string &manipName);
};

#endif // _OPENRAVESUPPORT_H_
