#pragma once
#include "environment.h"
#include "btBulletDynamicsCommon.h"
#include "vector"
using namespace std;
using boost::shared_ptr;

class CapsuleRope : public CompoundObject<BulletObject::Ptr> {
public:
  typedef shared_ptr<CapsuleRope> Ptr;
  vector<shared_ptr<btRigidBody> > bodies;
  vector<shared_ptr<btCollisionShape> > shapes;
  vector<shared_ptr<btTypedConstraint> > joints;
  btScalar radius;
  int nLinks;


public:
  CapsuleRope(const btAlignedObjectArray<btVector3>& ctrlPoints, float radius_);
  void init();
  void destroy();

};
