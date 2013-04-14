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

class BulletEnvironment;
class BULLETSIM_API BulletObject {
public:
  bool IsKinematic();
  string GetName();

  KinBodyPtr GetKinBody();
  py::object py_GetKinBody();

  btTransform GetTransform();
  py::object py_GetTransform();

  void SetTransform(const btTransform& t);
  void py_SetTransform(py::object py_hmat);

  void SetLinearVelocity(const btVector3& v);
  void py_SetLinearVelocity(py::list v);

  void SetAngularVelocity(const btVector3& w);
  void py_SetAngularVelocity(py::list w);

  void UpdateBullet();
  void UpdateRave();

private:
  friend class BulletEnvironment;
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

  void SetGravity(const btVector3& g);
  void py_SetGravity(py::list g);

  btVector3 GetGravity();
  py::object py_GetGravity();

  void Step(float dt, int maxSubSteps, float fixedTimeStep);

  vector<CollisionPtr> DetectAllCollisions();
  vector<CollisionPtr> ContactTest(BulletObjectPtr obj);

  void SetContactDistance(double dist);

private:
  Environment::Ptr m_env;
  RaveInstance::Ptr m_rave;
  vector<string> m_dynamic_obj_names;
  void init(EnvironmentBasePtr rave_env, const vector<string>& dynamic_obj_names);
};
typedef boost::shared_ptr<BulletEnvironment> BulletEnvironmentPtr;


} // namespace bs
