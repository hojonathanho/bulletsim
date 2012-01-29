#pragma once
#include "btBulletDynamicsCommon.h"
#include <openrave/openrave.h>
#include "bullet_typedefs.h"
#include <vector>
#include "basicobjects.h"
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

public:
    Monitor(OpenRAVE::RobotBase::ManipulatorPtr);
    virtual void update();
    virtual void grab() = 0;
    virtual void release() = 0;
    virtual void updateGrabPos() = 0;
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

    SoftMonitorForGrabbing(RaveRobotKinematicObject::Manipulator::Ptr manip, bool leftGripper) :
        Monitor(manip->manip),
        gripper(new PR2SoftBodyGripper(manip, leftGripper)) { }

    void setTarget(btSoftBody *psb) { gripper->setTarget(psb); }
    void grab() { gripper->grab(); }
    void release() { gripper->releaseAllAnchors(); }
    void updateGrabPos() { }
};
