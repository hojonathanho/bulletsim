#pragma once
#include "environment.h"
#include "basicobjects.h"
#include "btBulletDynamicsCommon.h"
#include "vector"
using namespace std;

class CapsuleRope : public CompoundObject<BulletObject> {
private:
  float angStiffness;
  float angDamping;
  float linDamping;
  float angLimit;
public:
  typedef boost::shared_ptr<CapsuleRope> Ptr;
  vector<boost::shared_ptr<btCollisionShape> > shapes;
  vector<BulletConstraint::Ptr> joints;
  btScalar radius;
  int nLinks;

  CapsuleRope(const vector<btVector3>& ctrlPoints, float radius_, float angStiffness_=.1, float angDamping_=1, float linDamping_=.75, float angLimit_=.4);
  void init();
  void destroy();
  vector<btVector3> getNodes() { 
    vector<btVector3> out(children.size());
    for (int i=0; i < children.size(); i++)
      out[i] = children[i]->rigidBody->getCenterOfMassPosition();
  return out;
  }
};
