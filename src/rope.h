#pragma once
#include "environment.h"
#include "btBulletDynamicsCommon.h"
#include "vector"
using namespace std;


class CapsuleRope : public EnvironmentObject {
public:
  typedef boost::shared_ptr<EnvironmentObject> Ptr;
  btAlignedObjectArray<btRigidBody*> bodies;
  btAlignedObjectArray<btTypedConstraint*> joints;
  btScalar radius;
  int nLinks;
  std::vector<BulletObject::Ptr> children;

public:
  CapsuleRope(const btAlignedObjectArray<btVector3>& ctrlPoints, float radius_);
  void init();
};
