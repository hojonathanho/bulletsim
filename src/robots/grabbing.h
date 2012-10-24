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

BulletObject::Ptr getNearestBody(const vector<BulletObject::Ptr>& bodies, const btVector3& pos, int& argmin);

class Monitor {
protected:
    bool m_wasClosed;
  RaveRobotObject::Manipulator::Ptr m_manip;

    //static const float PR2_CLOSED_VAL = 0.03f, PR2_OPEN_VAL = 0.54f;
    static const float PR2_CLOSED_VAL = 0.1f, PR2_OPEN_VAL = 0.54f;

    float closedThreshold;

public:
    typedef boost::shared_ptr<Monitor> Ptr;
    Monitor(); // must call setManip if using this constructor
  Monitor(RaveRobotObject::Manipulator::Ptr);
  virtual ~Monitor() {}
    void setClosedThreshold(float t) { closedThreshold = t; }

    virtual void update();
    virtual void grab() = 0;
    virtual void release() = 0;
    virtual void updateGrabPose() = 0;

  void setManip(RaveRobotObject::Manipulator::Ptr);
};

class MonitorForGrabbing : public Monitor {
public:
  typedef boost::shared_ptr<MonitorForGrabbing> Ptr;

  std::vector<BulletObject::Ptr> m_bodies;
  btDynamicsWorld* m_world;
  Grab* m_grab;
  int m_i;

  MonitorForGrabbing(RaveRobotObject::Manipulator::Ptr, btDynamicsWorld*);
  void setBodies(std::vector<BulletObject::Ptr>& bodies);
  void grab();
  void release();
  void updateGrabPose();
};

// softbody grabbing monitor
class SoftMonitorForGrabbing : public Monitor {
public:
  typedef boost::shared_ptr<SoftMonitorForGrabbing> Ptr;
    PR2SoftBodyGripper::Ptr gripper;

    SoftMonitorForGrabbing(RaveRobotObject::Ptr robot, RaveRobotObject::Manipulator::Ptr manip, bool leftGripper) :
        Monitor(manip),
        gripper(new PR2SoftBodyGripper(robot, manip->manip, leftGripper)) { }

    void setTarget(BulletSoftObject::Ptr sb) { gripper->setTarget(sb); }
    void grab() { gripper->grab(); }
    void release() { gripper->releaseAllAnchors(); }
    void updateGrabPose() { }
};
