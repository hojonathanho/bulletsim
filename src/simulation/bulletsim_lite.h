#pragma once

#include <boost/python.hpp>
#include "environment.h"
#include "openravesupport.h"
#include "macros.h"

namespace bs {

using namespace Eigen;
using namespace OpenRAVE;
namespace py = boost::python;

void InitPython();
struct SimulationParams; typedef boost::shared_ptr<SimulationParams> SimulationParamsPtr;
SimulationParamsPtr GetSimParams();
void TranslateStdException(const std::exception& e);

class BulletEnvironment;
class BULLETSIM_API BulletObject {
public:
  virtual ~BulletObject() { }

  bool IsKinematic();
  string GetName();

  KinBodyPtr GetKinBody();
  py::object py_GetKinBody();

  virtual btTransform GetTransform();
  virtual py::object py_GetTransform();

  virtual void SetTransform(const btTransform& t);
  virtual void py_SetTransform(py::object py_hmat);

  virtual void SetLinearVelocity(const btVector3& v);
  virtual void py_SetLinearVelocity(py::list v);

  virtual void SetAngularVelocity(const btVector3& w);
  virtual void py_SetAngularVelocity(py::list w);

  virtual void UpdateBullet();
  virtual void UpdateRave();

protected:
  friend class BulletEnvironment;
  BulletObject() { }
  BulletObject(RaveObject::Ptr obj) : m_obj(obj) { }
  RaveObject::Ptr m_obj;
};
typedef boost::shared_ptr<BulletObject> BulletObjectPtr;

struct Collision;
typedef boost::shared_ptr<Collision> CollisionPtr;
struct BULLETSIM_API Collision {
  KinBody::LinkPtr linkA;
  KinBody::LinkPtr linkB;
  btVector3 ptA, ptB, normalB2A;
  double distance;
  double weight;

  Collision(const KinBody::LinkPtr linkA_, const KinBody::LinkPtr linkB_, const btVector3& ptA_, const btVector3& ptB_, const btVector3& normalB2A_, double distance_, double weight_=1);

  py::object py_linkA();
  py::object py_linkB();
  py::object py_ptA();
  py::object py_ptB();
  py::object py_normalB2A();

  CollisionPtr Flipped() const;
};

struct BULLETSIM_API SimulationParams {
  float scale;
  btVector3 gravity;
  float dt;
  int maxSubSteps;
  float internalTimeStep;
  float friction;
  float restitution;
  float margin;
  float linkPadding;

  SimulationParams();
  void Apply();
};

class BULLETSIM_API BulletEnvironment {
public:
  BulletEnvironment(EnvironmentBasePtr rave_env, const vector<string>& dynamic_obj_names);
  // constructor for python interface
  BulletEnvironment(py::object py_rave_env, py::list dynamic_obj_names);

  ~BulletEnvironment();

  BulletObjectPtr GetObjectByName(const string &name);

  BulletObjectPtr GetObjectFromKinBody(KinBodyPtr kb);
  BulletObjectPtr py_GetObjectFromKinBody(py::object py_kb);

  vector<BulletObjectPtr> GetObjects();
  vector<BulletObjectPtr> GetDynamicObjects();

  EnvironmentBasePtr GetRaveEnv();
  py::object py_GetRaveEnv();

  Environment::Ptr GetBulletEnv();
  RaveInstance::Ptr GetRaveInstance();

  void SetGravity(const btVector3& g);
  void py_SetGravity(py::list g);

  btVector3 GetGravity();
  py::object py_GetGravity();

  void Step(float dt, int maxSubSteps, float fixedTimeStep);

  vector<CollisionPtr> DetectAllCollisions();
  vector<CollisionPtr> ContactTest(BulletObjectPtr obj);

  void SetContactDistance(double dist);

  BulletConstraint::Ptr AddConstraint(BulletConstraint::Ptr cnt);
  BulletConstraint::Ptr py_AddConstraint(py::dict desc);
  void RemoveConstraint(BulletConstraint::Ptr cnt);

private:
  Environment::Ptr m_env;
  RaveInstance::Ptr m_rave;
  vector<string> m_dynamic_obj_names;
  void init(EnvironmentBasePtr rave_env, const vector<string>& dynamic_obj_names);
};
typedef boost::shared_ptr<BulletEnvironment> BulletEnvironmentPtr;


struct BULLETSIM_API CapsuleRopeParams {
  float radius;
  float angStiffness;
  float angDamping;
  float linDamping;
  float angLimit;
  float linStopErp;
};

class BULLETSIM_API CapsuleRope : public BulletObject {
public:
  CapsuleRope(BulletEnvironmentPtr env, const string& name, const vector<btVector3>& ctrlPoints, const CapsuleRopeParams& params);
  CapsuleRope(BulletEnvironmentPtr env, const string& name, py::object ctrlPoints, const CapsuleRopeParams& params); // boost python wrapper

  CapsuleRopeParams m_params;

  virtual void UpdateRave();

  std::vector<btVector3> GetNodes();
  std::vector<btVector3> GetControlPoints();
  vector<btMatrix3x3> GetRotations();
  vector<float> GetHalfHeights();

  py::object py_GetNodes();
  py::object py_GetControlPoints();
  py::object py_GetRotations();
  py::object py_GetHalfHeights();

  // not supported
  virtual void UpdateBullet();
  virtual void SetTransform(const btTransform&);
  virtual void SetLinearVelocity(const btVector3&);
  virtual void SetAngularVelocity(const btVector3&);
  // end not supported

private:
  vector<RaveLinkObject::Ptr> m_children;
  vector<btRigidBody*> m_children_rigidbodies;

  void init(BulletEnvironmentPtr env, const string& name, const vector<btVector3>& ctrlPoints, const CapsuleRopeParams& params);
};
typedef boost::shared_ptr<CapsuleRope> CapsuleRopePtr;

} // namespace bs
