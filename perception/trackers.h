#pragma once
#include "bullet_typedefs.h"
#include "basicobjects.h"
#include <boost/shared_ptr.hpp>
#include <vector>
using boost::shared_ptr;

class MultiPointTrackerRigid {
public:
  int m_N;
  vector<P2PConstraintPtr> m_constraints;
  vector<RigidBodyPtr> m_bodies;
  vector<bool> m_active;
  btDynamicsWorld* m_world;

  MultiPointTrackerRigid(vector<RigidBodyPtr>&, btDynamicsWorld*);
  void update(const vector<btVector3>& obsPts);
  void setActive(int, bool);
  void updatePos(int, const btVector3&);
};

class TrackerPlotter {
public:
  int m_N;
  MultiPointTrackerRigid* m_tracker;
  vector<BulletObject::Ptr> m_fakeObjects; // todo: just use osg objects
  TrackerPlotter(MultiPointTrackerRigid& tracker);
  void update();
};

