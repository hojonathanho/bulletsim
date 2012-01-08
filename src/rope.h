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


public:
  CapsuleRope(const vector<btVector3>& ctrlPoints, float radius_, float angStiffness=.1, float angDamping=1, float linDamping=.75, float angLimit=.4);
  void init();
  void destroy();
  void getPts(vector<btVector3>& centers) { 
    centers.resize(bodies.size());
    for (int i=0; i < bodies.size(); i++) 
      centers[i] = bodies[i]->getCenterOfMassPosition();
  }
};
