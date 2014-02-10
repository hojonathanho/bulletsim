#pragma once
#include "environment.h"
#include "basicobjects.h"
#include "btBulletDynamicsCommon.h"
#include <vector>


boost::shared_ptr<btGeneric6DofSpringConstraint> CapsuleRope_createBendConstraint(btScalar len, const boost::shared_ptr<btRigidBody> rbA, const boost::shared_ptr<btRigidBody>& rbB, float damping, float stiffness, float limit);
btMatrix3x3 CapsuleRope_makePerpBasis(const btVector3& a0);
void CapsuleRope_createRopeTransforms(vector<btTransform>& transforms, vector<btScalar>& lengths, const vector<btVector3>& ctrlPoints);
vector<btVector3> CapsuleRope_getNodes(const vector<btRigidBody*> &capsules);
vector<btVector3> CapsuleRope_getControlPoints(const vector<btRigidBody*> &capsules);
vector<btMatrix3x3> CapsuleRope_getRotations(const vector<btRigidBody*> &capsules);
void CapsuleRope_setRotations(const vector<btRigidBody*> &capsules, const vector<btMatrix3x3>& rots);
vector<btVector3> CapsuleRope_getTranslations(const vector<btRigidBody*> &capsules);
void CapsuleRope_setTranslations(const vector<btRigidBody*> &capsules, const vector<btVector3>& trans);
vector<float> CapsuleRope_getHalfHeights(const vector<btRigidBody*> &capsules);

class CapsuleRope : public CompoundObject<BulletObject> {
private:
  float angStiffness;
  float angDamping;
  float linDamping;
  float angLimit;
  std::vector<btRigidBody*> children_rigidBodies;
public:
  typedef boost::shared_ptr<CapsuleRope> Ptr;
  std::vector<boost::shared_ptr<btCollisionShape> > shapes;
  std::vector<BulletConstraint::Ptr> joints;
  btScalar radius;
  int nLinks;

  CapsuleRope(const std::vector<btVector3>& ctrlPoints, float radius_, float angStiffness_=.1, float angDamping_=1, float linDamping_=.75, float angLimit_=.4, float linStopErp_=.2);
  void init();
  void destroy();

  std::vector<btVector3> getNodes();
  std::vector<btVector3> getControlPoints();
  vector<btMatrix3x3> getRotations();
  void setRotations(const vector<btMatrix3x3>& rots);
  vector<btVector3> getTranslations();
  void setTranslations(const vector<btVector3>& trans);
  vector<float> getHalfHeights();
};
