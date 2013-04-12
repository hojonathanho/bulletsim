#include "lite.h"

//#include <boost/python.hpp>
//#include "environment.h"
//#include "openravesupport.h"
#include "logging.h"

namespace bs {

static py::object openravepy, numpy;

void InitPython() {
  openravepy = py::import("openravepy");
  numpy = py::import("numpy");
}

vector<string> toStrVec(py::list py_str_list) {
  int n = py::len(py_str_list);
  vector<string> out;
  for (int i = 0; i < n; ++i) {
    out.push_back(py::extract<string>(py_str_list[i]));
  }
  return out;
}

btVector3 toBtVector3(py::list v) {
  assert(py::len(v) == 3);
  return btVector3(py::extract<btScalar>(v[0]), py::extract<btScalar>(v[1]), py::extract<btScalar>(v[2]));
}

template<typename T>
struct type_traits {
  static const char* npname;
};
template<> const char* type_traits<float>::npname = "float32";
template<> const char* type_traits<int>::npname = "int32";
template<> const char* type_traits<double>::npname = "float64";

template <typename T>
T* getPointer(const py::object& arr) {
  long int i = py::extract<long int>(arr.attr("ctypes").attr("data"));
  T* p = (T*)i;
  return p;
}

template<typename T>
py::object toNdarray1(const T* data, size_t dim0) {
  py::object out = numpy.attr("empty")(py::make_tuple(dim0), type_traits<T>::npname);
  T* p = getPointer<T>(out);
  memcpy(p, data, dim0*sizeof(T));
  return out;
}
template<typename T>
py::object toNdarray2(const T* data, size_t dim0, size_t dim1) {
  py::object out = numpy.attr("empty")(py::make_tuple(dim0, dim1), type_traits<T>::npname);
  T* pout = getPointer<T>(out);
  memcpy(pout, data, dim0*dim1*sizeof(T));
  return out;
}

py::object toNdarray(const btVector3 &v) {
  return toNdarray1(v.m_floats, 3); // TODO: check float vs double
}

template<typename T>
py::object ensureFormat(py::object ndarray) {
  // ensure C-order and data type, possibly making a new ndarray
  return numpy.attr("ascontiguousarray")(ndarray, type_traits<T>::npname);
}

template<typename T>
void fromNdarray2(py::object a, vector<T> &out, size_t &out_dim0, size_t &out_dim1) {
  a = ensureFormat<T>(a);
  py::object shape = a.attr("shape");
  if (py::len(shape) != 2) {
    throw std::runtime_error((boost::format("expected 2-d array, got %d-d instead") % py::len(shape)).str());
  }
  out_dim0 = py::extract<size_t>(shape[0]);
  out_dim1 = py::extract<size_t>(shape[1]);
  out.resize(out_dim0 * out_dim1);
  memcpy(out.data(), getPointer<T>(a), out_dim0*out_dim1*sizeof(T));
}

template<typename KeyT, typename ValueT>
ValueT &findOrFail(map<KeyT, ValueT> &m, const KeyT &key, const string &error_str="") {
  typename map<KeyT, ValueT>::iterator i = m.find(key);
  if (i == m.end()) {
    throw std::runtime_error(error_str);
  }
  return i->second;
}


EnvironmentBasePtr GetCppEnv(py::object py_env) {
  int id = py::extract<int>(openravepy.attr("RaveGetEnvironmentId")(py_env));
  EnvironmentBasePtr cpp_env = RaveGetEnvironment(id);
  return cpp_env;
}
py::object GetPyEnv(EnvironmentBasePtr cpp_env) {
  int id = RaveGetEnvironmentId(cpp_env);
  return openravepy.attr("RaveGetEnvironment")(id);
}
KinBodyPtr GetCppKinBody(py::object py_kb, EnvironmentBasePtr env) {
  int id = py::extract<int>(py_kb.attr("GetEnvironmentId")());
  return env->GetBodyFromEnvironmentId(id);
}
py::object GetPyKinBody(KinBodyPtr kb) {
  int id = kb->GetEnvironmentId();
  return GetPyEnv(kb->GetEnv()).attr("GetBodyFromEnvironmentId")(id);
}
KinBody::LinkPtr GetCppLink(py::object py_link, EnvironmentBasePtr env) {
  KinBodyPtr parent = GetCppKinBody(py_link.attr("GetParent")(), env);
  int idx = py::extract<int>(py_link.attr("GetIndex")());
  return parent->GetLinks()[idx];
}
py::object GetPyLink(KinBody::LinkPtr link) {
  py::object parent = GetPyKinBody(link->GetParent());
  return parent.attr("GetLinks")()[link->GetIndex()];
}


bool BulletObject::IsKinematic() { return m_obj->getIsKinematic(); }
string BulletObject::GetName() { return m_obj->body->GetName(); }
KinBodyPtr BulletObject::GetKinBody() { return m_obj->body; }
py::object BulletObject::py_GetKinBody() { return GetPyKinBody(m_obj->body); }

btTransform BulletObject::GetTransform() {
  return m_obj->toRaveFrame(m_obj->children[0]->rigidBody->getCenterOfMassTransform());
}
py::object BulletObject::py_GetTransform() {
  btScalar mat[16];
  GetTransform().getOpenGLMatrix(mat);
  return toNdarray2(mat, 4, 4).attr("T");
}

void BulletObject::SetTransform(const btTransform& t) {
  //m_obj->children[0]->rigidBody->setCenterOfMassTransform(m_obj->toWorldFrame(t));
  m_obj->children[0]->motionState->setKinematicPos(m_obj->toWorldFrame(t));
}
void BulletObject::py_SetTransform(py::object py_hmat) {
  vector<btScalar> hmat; size_t dim0, dim1;
  fromNdarray2(py_hmat.attr("T"), hmat, dim0, dim1);
  if (dim0 != 4 || dim1 != 4) {
    throw std::runtime_error((boost::format("expected 4x4 matrix, got %dx%d") % dim0 % dim1).str());
  }
  btTransform t;
  t.setFromOpenGLMatrix(hmat.data());
  SetTransform(t);
}

void BulletObject::SetLinearVelocity(const btVector3& v) {
  m_obj->children[0]->rigidBody->setLinearVelocity(v);
}
void BulletObject::py_SetLinearVelocity(py::list v) {
  SetLinearVelocity(toBtVector3(v));
}

void BulletObject::SetAngularVelocity(const btVector3& w) {
  m_obj->children[0]->rigidBody->setAngularVelocity(w);
}
void BulletObject::py_SetAngularVelocity(py::list w) {
  SetAngularVelocity(toBtVector3(w));
}

void BulletObject::UpdateBullet() {
  m_obj->updateBullet();
}

void BulletObject::UpdateRave() {
  m_obj->updateRave();
}


Collision::Collision(const KinBody::LinkPtr linkA_, const KinBody::LinkPtr linkB_, const btVector3& ptA_, const btVector3& ptB_, const btVector3& normalB2A_, double distance_, double weight_) :
  linkA(linkA_),
  linkB(linkB_),
  ptA(ptA_),
  ptB(ptB_),
  normalB2A(normalB2A_),
  distance(distance_),
  weight(weight_)
{ }

py::object Collision::py_linkA() { return GetPyLink(linkA); }
py::object Collision::py_linkB() { return GetPyLink(linkB); }
py::object Collision::py_ptA() { return toNdarray(ptA); }
py::object Collision::py_ptB() { return toNdarray(ptB); }
py::object Collision::py_normalB2A() { return toNdarray(normalB2A); }

CollisionPtr Collision::Flipped() const {
  return CollisionPtr(new Collision(linkB, linkA, ptB, ptA, -normalB2A, distance, weight));
}



void BulletEnvironment::init(EnvironmentBasePtr rave_env, const vector<string>& dynamic_obj_names) {
  BulletInstance::Ptr bullet(new BulletInstance);
  m_env.reset(new Environment(bullet));
  m_rave.reset(new RaveInstance(rave_env));
  m_dynamic_obj_names = dynamic_obj_names;
  LoadFromRaveExplicit(m_env, m_rave, dynamic_obj_names);
  m_env->bullet->setGravity(btVector3(0, 0, -9.8));
}

BulletEnvironment::BulletEnvironment(EnvironmentBasePtr rave_env, const vector<string>& dynamic_obj_names) {
  init(rave_env, dynamic_obj_names);
}

BulletEnvironment::BulletEnvironment(py::object py_rave_env, py::list dynamic_obj_names) {
  init(GetCppEnv(py_rave_env), toStrVec(dynamic_obj_names));
}

BulletEnvironment::~BulletEnvironment() {
  LOG_DEBUG("py bullet env destroyed");
}

BulletObjectPtr BulletEnvironment::GetObjectByName(const string &name) {
  return BulletObjectPtr(new BulletObject(getObjectByName(m_env, m_rave, name)));
}

BulletObjectPtr BulletEnvironment::GetObjectFromKinBody(KinBodyPtr kb) {
  if (RaveGetEnvironmentId(kb->GetEnv()) != RaveGetEnvironmentId(m_rave->env)) {
    throw std::runtime_error("trying to get Bullet object for a KinBody that doesn't belong to this (OpenRAVE base) environment");
  }
  return BulletObjectPtr(new BulletObject(getObjectByName(m_env, m_rave, kb->GetName())));
}
BulletObjectPtr BulletEnvironment::py_GetObjectFromKinBody(py::object py_kb) {
  if (openravepy.attr("RaveGetEnvironmentId")(py_kb.attr("GetEnv")()) != RaveGetEnvironmentId(m_rave->env)) {
    throw std::runtime_error("trying to get Bullet object for a KinBody that doesn't belong to this (OpenRAVE base) environment");
  }
  return BulletObjectPtr(new BulletObject(getObjectByName(m_env, m_rave, GetCppKinBody(py_kb, m_rave->env)->GetName())));
}

vector<BulletObjectPtr> BulletEnvironment::GetObjects() {
  vector<KinBodyPtr> bodies; m_rave->env->GetBodies(bodies);
  vector<BulletObjectPtr> out; out.reserve(bodies.size());
  BOOST_FOREACH(const KinBodyPtr& body, bodies) {
    out.push_back(GetObjectByName(body->GetName()));
  }
  return out;
}

vector<BulletObjectPtr> BulletEnvironment::GetDynamicObjects() {
  vector<BulletObjectPtr> out;
  out.reserve(m_dynamic_obj_names.size());
  BOOST_FOREACH(const string& name, m_dynamic_obj_names) {
    out.push_back(GetObjectByName(name));
  }
  return out;
}

EnvironmentBasePtr BulletEnvironment::GetRaveEnv() {
  return m_rave->env;
}
py::object BulletEnvironment::py_GetRaveEnv() {
  return GetPyEnv(m_rave->env);
}

void BulletEnvironment::SetGravity(const btVector3& g) {
  m_env->bullet->setGravity(g);
}
void BulletEnvironment::py_SetGravity(py::list g) {
  SetGravity(toBtVector3(g));
}

btVector3 BulletEnvironment::GetGravity() {
  return m_env->bullet->dynamicsWorld->getGravity();
}
py::object BulletEnvironment::py_GetGravity() {
  return toNdarray(GetGravity());
}

void BulletEnvironment::Step(float dt, int maxSubSteps, float fixedTimeStep) {
  m_env->step(dt, maxSubSteps, fixedTimeStep);
}

vector<CollisionPtr> BulletEnvironment::DetectCollisions() {
  vector<CollisionPtr> collisions;
  btDynamicsWorld *world = m_env->bullet->dynamicsWorld;
  btCollisionDispatcher *dispatcher = m_env->bullet->dispatcher;
  //world->performDiscreteCollisionDetection();
  int numManifolds = dispatcher->getNumManifolds();
  LOG_DEBUG_FMT("number of manifolds: %i", numManifolds);
  for (int i = 0; i < numManifolds; ++i) {
    btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);
    int numContacts = contactManifold->getNumContacts();
    LOG_DEBUG_FMT("number of contacts in manifold %i: %i", i, numContacts);
    btRigidBody *objA = static_cast<btRigidBody *>(contactManifold->getBody0());
    btRigidBody *objB = static_cast<btRigidBody *>(contactManifold->getBody1());
    for (int j = 0; j < numContacts; ++j) {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);
      KinBody::LinkPtr linkA = findOrFail(m_rave->bulletsim2rave_links, objA);
      KinBody::LinkPtr linkB = findOrFail(m_rave->bulletsim2rave_links, objB);
      collisions.push_back(CollisionPtr(new Collision(
        linkA, linkB, pt.getPositionWorldOnA(), pt.getPositionWorldOnB(),
        pt.m_normalWorldOnB, pt.m_distance1, 1./numContacts)));
      LOG_DEBUG_FMT("%s/%s - %s/%s collided", linkA->GetParent()->GetName().c_str(), linkA->GetName().c_str(), linkB->GetParent()->GetName().c_str(), linkB->GetName().c_str());
    }
    // caching helps performance, but for optimization the cost should not be history-dependent
    //contactManifold->clearManifold();
  }
  return collisions;
}

void BulletEnvironment::SetContactDistance(double dist) {
  LOG_DEBUG_FMT("setting contact distance to %.2f", dist);
  //m_contactDistance = dist;
  //SHAPE_EXPANSION = btVector3(1,1,1)*dist;
  //gContactBreakingThreshold = 2.001*dist; // wtf. when I set it to 2.0 there are no contacts with distance > 0
  btCollisionObjectArray& objs = m_env->bullet->dynamicsWorld->getCollisionObjectArray();
  for (int i = 0; i < objs.size(); ++i) {
    objs[i]->setContactProcessingThreshold(dist);
  }
  btCollisionDispatcher* dispatcher = m_env->bullet->dispatcher;
  dispatcher->setDispatcherFlags(dispatcher->getDispatcherFlags() & ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);
}


} // namespace bs
