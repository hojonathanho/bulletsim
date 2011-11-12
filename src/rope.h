#pragma once
#include "environment.h"
#include "btBulletDynamicsCommon.h"
#include "vector"
using namespace std;


class CapsuleRope : public CompoundObject<BulletObject::Ptr> {
public:
  typedef boost::shared_ptr<CapsuleRope> Ptr;
  btAlignedObjectArray<btRigidBody*> bodies;
  btAlignedObjectArray<btTypedConstraint*> joints;
  btScalar radius;
  int nLinks;
  std::vector<BulletObject::Ptr> children;

public:
  CapsuleRope(const btAlignedObjectArray<btVector3>& ctrlPoints, float radius_);
  void init();
  //  void prePhysics();
  //void preDraw();
};
