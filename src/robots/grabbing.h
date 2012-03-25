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
  Grab(btRigidBody* rb, const btVector3& pos, btDynamicsWorld* world);
  Grab(btRigidBody* rb, const btTransform& pose, btDynamicsWorld* world);
  ~Grab();

  void updatePosition(const btVector3& pos);
  void updatePose(const btTransform& pose);

};

BulletObject::Ptr getNearestBody(vector<BulletObject::Ptr> bodies, btVector3 pos, int argmin);

class Monitor {
protected:
    bool m_wasClosed;
  RaveRobotObject::Manipulator::Ptr m_manip;

    //static const float PR2_CLOSED_VAL = 0.03f, PR2_OPEN_VAL = 0.54f;
    static const float PR2_CLOSED_VAL = 0.1f, PR2_OPEN_VAL = 0.54f;

    float closedThreshold;

public:
    Monitor(); // must call setManip if using this constructor
  Monitor(RaveRobotObject::Manipulator::Ptr);

    void setClosedThreshold(float t) { closedThreshold = t; }

    virtual void update();
    virtual void grab() = 0;
    virtual void release() = 0;
    virtual void updateGrabPose() = 0;

  void setManip(RaveRobotObject::Manipulator::Ptr);
};

class MonitorForGrabbing : public Monitor {
public:
  std::vector<BulletObject::Ptr> m_bodies;
  btDynamicsWorld* m_world;
  Grab* m_grab;
  int m_i;

  MonitorForGrabbing(RaveRobotObject::Manipulator::Ptr, btDynamicsWorld*);
  void setBodies(std::vector<BulletObject::Ptr>& bodies);
  virtual void grab();
  virtual void release();
  virtual void updateGrabPose();
};

// softbody grabbing monitor
class SoftMonitorForGrabbing : public Monitor {
public:
    PR2SoftBodyGripper::Ptr gripper;

  SoftMonitorForGrabbing(RaveRobotObject::Ptr robot, OpenRAVE::RobotBase::ManipulatorPtr manip, bool leftGripper);
    void setTarget(btSoftBody *psb) { gripper->setTarget(psb); }
    void grab() { gripper->grab(); }
    void release() { gripper->releaseAllAnchors(); }
    void updateGrabPos() { }
};
