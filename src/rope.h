#pragma once
#include "environment.h"
#include "basicobjects.h"
#include "btBulletDynamicsCommon.h"
#include "vector"
using namespace std;
using boost::shared_ptr;

class CapsuleRope : public CompoundObject<BulletObject> {
private:
  float angStiffness;
  float angDamping;
  float linDamping;
  float angLimit;
public:
  typedef shared_ptr<CapsuleRope> Ptr;
  vector<shared_ptr<btRigidBody> > bodies;
  vector<shared_ptr<btCollisionShape> > shapes;
  vector<shared_ptr<btTypedConstraint> > joints;
  btScalar radius;
  int nLinks;

  CapsuleRope(const vector<btVector3>& ctrlPoints, float radius_, float angStiffness_=.1, float angDamping_=1, float linDamping_=.75, float angLimit_=.4);
  void init();
  void destroy();
  vector<btVector3> getNodes() { 
    vector<btVector3> out(bodies.size());
    for (int i=0; i < bodies.size(); i++) 
      out[i] = bodies[i]->getCenterOfMassPosition();
  return out;
  }
};
