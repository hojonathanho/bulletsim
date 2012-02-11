#pragma once
#include "btBulletDynamicsCommon.h"
#include <openrave/openrave.h>
#include <vector>
#include "simulation/basicobjects.h"
#include "simulation/bullet_typedefs.h"
#include "pr2.h"

class Grab {
public:
  btGeneric6DofConstraint* cnt;
  btDynamicsWorld* world;
  Grab(){}
  Grab(btRigidBody* rb, btVector3 pos, btDynamicsWorld* world);
  ~Grab();

  void updatePosition(btVector3 pos);
};

class Monitor {
protected:
    bool m_wasClosed;
    OpenRAVE::RobotBase::ManipulatorPtr m_manip;

    //static const float PR2_CLOSED_VAL = 0.03f, PR2_OPEN_VAL = 0.54f;
    static const float PR2_CLOSED_VAL = 0.1f, PR2_OPEN_VAL = 0.54f;

    float closedThreshold;

public:
    Monitor(); // must call setManip if using this constructor
    Monitor(OpenRAVE::RobotBase::ManipulatorPtr);

    void setClosedThreshold(float t) { closedThreshold = t; }

    virtual void update();
    virtual void grab() = 0;
    virtual void release() = 0;
    virtual void updateGrabPos() = 0;

    void setManip(OpenRAVE::RobotBase::ManipulatorPtr);
};

class MonitorForGrabbing : public Monitor {
public:
  std::vector<BulletObject::Ptr> m_bodies;
  btDynamicsWorld* m_world;
  Grab* m_grab;

  MonitorForGrabbing(OpenRAVE::RobotBase::ManipulatorPtr, btDynamicsWorld*);
  void setBodies(std::vector<BulletObject::Ptr>& bodies);
  void grab();
  void release();
  void updateGrabPos();
};

// softbody grabbing monitor
class SoftMonitorForGrabbing : public Monitor {
public:
    PR2SoftBodyGripper::Ptr gripper;

    SoftMonitorForGrabbing(RaveRobotKinematicObject::Ptr robot, OpenRAVE::RobotBase::ManipulatorPtr manip, bool leftGripper) :
        Monitor(manip),
        gripper(new PR2SoftBodyGripper(robot, manip, leftGripper)) { }
    SoftMonitorForGrabbing(RaveRobotKinematicObject::Ptr robot, bool leftGripper);

    void setTarget(btSoftBody *psb) { gripper->setTarget(psb); }
    void grab() { gripper->grab(); }
    void release() { gripper->releaseAllAnchors(); }
    void updateGrabPos() { }
};
