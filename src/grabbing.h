#pragma once
#include "btBulletDynamicsCommon.h"
#include <openrave/openrave.h>
#include "bullet_typedefs.h"
#include <vector>
#include "basicobjects.h"

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
    OpenRAVE::RobotBase::ManipulatorPtr manip;
    BulletInstance::Ptr bullet;

    bool m_wasClosed;

public:
    Monitor(OpenRAVE::RobotBase::ManipulatorPtr manip_, BulletInstance::Ptr bullet_) :
        manip(manip_), bullet(bullet_) { }
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

  MonitorForGrabbing(OpenRAVE::RobotBase::ManipulatorPtr, BulletInstance::Ptr);
  void setBodies(std::vector<BulletObject::Ptr>& bodies);
  void grab();
  void release();
  void updateGrabPos();
};

// softbody grabbing monitor
class SoftMonitorForGrabbing : public Monitor {
public:
    btSoftBody *psb;

    SoftMonitorForGrabbing(OpenRAVE::RobotBase::ManipulatorPtr m, BulletInstance::Ptr bullet) :
        Monitor(m, bullet) { }

    void update();
};
