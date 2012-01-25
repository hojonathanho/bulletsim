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

class MonitorForGrabbing {
public:
  OpenRAVE::RobotBase::ManipulatorPtr m_manip;
  std::vector<BulletObject::Ptr> m_bodies;
  btDynamicsWorld* m_world;
  bool m_wasClosed;
  Grab* m_grab;


  MonitorForGrabbing(OpenRAVE::RobotBase::ManipulatorPtr, btDynamicsWorld* world);
  void update();
  void setBodies(std::vector<BulletObject::Ptr>& bodies);
  void grabNearestObject();
  void releaseObject();
};
