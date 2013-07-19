#include "bulletsim_lite.h"
#include "logging.h"

#include "rope.h"

namespace bs {

static py::object openravepy, numpy;
static SimulationParamsPtr g_simparams;

void InitPython() {
  openravepy = py::import("openravepy");
  numpy = py::import("numpy");
}

SimulationParamsPtr GetSimParams() {
  if (!g_simparams) {
    g_simparams.reset(new SimulationParams);
  }
  return g_simparams;
}

void TranslateStdException(const std::exception& e) {
  PyErr_SetString(PyExc_RuntimeError, e.what());
}

vector<string> toStrVec(py::list py_str_list) {
  int n = py::len(py_str_list);
  vector<string> out;
  for (int i = 0; i < n; ++i) {
    out.push_back(py::extract<string>(py_str_list[i]));
  }
  return out;
}

string toStr(const btVector3 &v) {
  stringstream ss;
  ss << v.x() << ' ' << v.y() << ' ' << v.z();
  return ss.str();
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
py::object toNdarray(const vector<T> &v) {
  return toNdarray1<T>(v.data(), v.size());
}

py::object toNdarray2(const vector<btVector3> &vs) {
  py::object out = numpy.attr("empty")(py::make_tuple(vs.size(), 3), type_traits<btScalar>::npname);
  btScalar* pout = getPointer<btScalar>(out);
  for (int i = 0; i < vs.size(); ++i) {
    for (int j = 0; j < 3; ++j) {
      *(pout + 3*i + j) = vs[i].m_floats[j];
    }
  }
  return out;
}

py::object toNdarray3(const vector<btMatrix3x3> &v) {
  py::object out = numpy.attr("empty")(py::make_tuple(v.size(), 3, 3), type_traits<btScalar>::npname);
  btScalar* pout = getPointer<btScalar>(out);
  for (int i = 0; i < v.size(); ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        *(pout + 9*i + 3*j + k) = v[i].getRow(j).m_floats[k];
      }
    }
  }
  return out;
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

btVector3 fromNdarray1ToBtVec3(py::object a) {
  a = ensureFormat<btScalar>(a);
  py::object shape = a.attr("shape");
  if (py::len(shape) != 1) {
    throw std::runtime_error((boost::format("expected 1-d array, got %d-d instead") % py::len(shape)).str());
  }
  size_t out_dim0 = py::extract<size_t>(shape[0]);
  if (out_dim0 != 3) {
    throw std::runtime_error((boost::format("expected shape[0] == 3, got %d instead") % out_dim0).str());
  }

  btScalar* pin = getPointer<btScalar>(a);
  return btVector3(*pin, *(pin + 1), *(pin + 2));
}

void fromNdarray2ToBtVecs(py::object a, vector<btVector3> &out) {
  a = ensureFormat<btScalar>(a);
  py::object shape = a.attr("shape");
  if (py::len(shape) != 2) {
    throw std::runtime_error((boost::format("expected 2-d array, got %d-d instead") % py::len(shape)).str());
  }
  size_t out_dim0 = py::extract<size_t>(shape[0]);
  size_t out_dim1 = py::extract<size_t>(shape[1]);
  if (out_dim1 != 3) {
    throw std::runtime_error((boost::format("expected shape[1] == 3, got %d instead") % out_dim1).str());
  }

  btScalar* pin = getPointer<btScalar>(a);
  out.resize(out_dim0);
  for (int i = 0; i < out_dim0; ++i) {
    for (int j = 0; j < 3; ++j) {
      out[i].m_floats[j] = *(pin + 3*i + j);
    }
  }
}

btVector3 toBtVector3(py::object v) {
  py::extract<py::list> get_list(v);
  if (get_list.check()) {
    py::list l = get_list();
    if (py::len(l) != 3) {
      throw std::runtime_error((boost::format("Expected list of length 3, got %d instead") % py::len(l)).str());
    }
    return btVector3(py::extract<btScalar>(l[0]), py::extract<btScalar>(l[1]), py::extract<btScalar>(l[2]));
  }
  return fromNdarray1ToBtVec3(v);
}

btTransform toBtTransform(py::object py_hmat, btScalar scale=1) {
  vector<btScalar> hmat; size_t dim0, dim1;
  fromNdarray2(py_hmat.attr("T"), hmat, dim0, dim1);
  if (dim0 != 4 || dim1 != 4) {
    throw std::runtime_error((boost::format("expected 4x4 matrix, got %dx%d") % dim0 % dim1).str());
  }
  btTransform t;
  t.setFromOpenGLMatrix(hmat.data());
  t.getOrigin() *= scale;
  return t;
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
  SetTransform(toBtTransform(py_hmat, 1));
}

void BulletObject::SetLinearVelocity(const btVector3& v) {
  m_obj->children[0]->rigidBody->setLinearVelocity(v * METERS);
}
void BulletObject::py_SetLinearVelocity(py::list v) {
  SetLinearVelocity(toBtVector3(v));
}

void BulletObject::SetAngularVelocity(const btVector3& w) {
  m_obj->children[0]->rigidBody->setAngularVelocity(w * METERS);
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


SimulationParams::SimulationParams()
  : scale(10.),
    gravity(btVector3(0, 0, -1)),
    dt(.01),
    maxSubSteps(200),
    internalTimeStep(1./200.),
    friction(.5),
    restitution(0),
    margin(.0005),
    linkPadding(0)
{ }

void SimulationParams::Apply() {
  GeneralConfig::scale = scale;
  BulletConfig::gravity = gravity;
  BulletConfig::dt = dt;
  BulletConfig::maxSubSteps = maxSubSteps;
  BulletConfig::internalTimeStep = internalTimeStep;
  BulletConfig::friction = friction;
  BulletConfig::restitution = restitution;
  BulletConfig::margin = margin;
  BulletConfig::linkPadding = linkPadding;
}

void BulletEnvironment::init(EnvironmentBasePtr rave_env, const vector<string>& dynamic_obj_names) {
  GetSimParams()->Apply();
  BulletInstance::Ptr bullet(new BulletInstance);
  m_env.reset(new Environment(bullet));
  m_rave.reset(new RaveInstance(rave_env));
  m_dynamic_obj_names = dynamic_obj_names;
  LoadFromRaveExplicit(m_env, m_rave, dynamic_obj_names);
  SetGravity(GetSimParams()->gravity);
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

Environment::Ptr BulletEnvironment::GetBulletEnv() {
  return m_env;
}

RaveInstance::Ptr BulletEnvironment::GetRaveInstance() {
  return m_rave;
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

vector<CollisionPtr> BulletEnvironment::DetectAllCollisions() {
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
        linkA, linkB, pt.getPositionWorldOnA()/METERS, pt.getPositionWorldOnB()/METERS,
        pt.m_normalWorldOnB/METERS, pt.m_distance1/METERS, 1./numContacts)));
      LOG_DEBUG_FMT("%s/%s - %s/%s collided", linkA->GetParent()->GetName().c_str(), linkA->GetName().c_str(), linkB->GetParent()->GetName().c_str(), linkB->GetName().c_str());
    }
    // caching helps performance, but for optimization the cost should not be history-dependent
    //contactManifold->clearManifold();
  }
  return collisions;
}

vector<CollisionPtr> BulletEnvironment::ContactTest(BulletObjectPtr obj) {
  vector<CollisionPtr> out;
  struct ContactCallback : public btCollisionWorld::ContactResultCallback {
    vector<CollisionPtr> &m_out;
    RaveInstance::Ptr m_rave;
    ContactCallback(vector<CollisionPtr> &out_, RaveInstance::Ptr rave) : m_out(out_), m_rave(rave) { }
    btScalar addSingleResult(btManifoldPoint &pt,
                             const btCollisionObject *colObj0, int, int,
                             const btCollisionObject *colObj1, int, int) {
      btRigidBody *objA = const_cast<btRigidBody *>(static_cast<const btRigidBody *>(colObj0));
      btRigidBody *objB = const_cast<btRigidBody *>(static_cast<const btRigidBody *>(colObj1));
      KinBody::LinkPtr linkA = findOrFail(m_rave->bulletsim2rave_links, objA);
      KinBody::LinkPtr linkB = findOrFail(m_rave->bulletsim2rave_links, objB);
      m_out.push_back(CollisionPtr(new Collision(
        linkA, linkB, pt.getPositionWorldOnA()/METERS, pt.getPositionWorldOnB()/METERS,
        pt.m_normalWorldOnB/METERS, pt.m_distance1/METERS, 1.)));
      return 0;
    }
  } cb(out, m_rave);

  // do contact test for all links of obj
  RaveObject::ChildVector& obj_children = obj->m_obj->getChildren();
  for (int i = 0; i < obj_children.size(); ++i) {
    m_env->bullet->dynamicsWorld->contactTest(obj_children[i]->rigidBody.get(), cb);
  }

  return out;
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


BulletConstraint::Ptr BulletEnvironment::AddConstraint(BulletConstraint::Ptr cnt) {
  m_env->addConstraint(cnt);
  return cnt;
}

BulletConstraint::Ptr BulletEnvironment::py_AddConstraint(py::dict desc) {
  string type = py::extract<string>(desc["type"]);
  py::dict params = py::extract<py::dict>(desc["params"]);

  btTypedConstraint *cnt;
  if (type == "point2point") {
    KinBody::LinkPtr linkA = GetCppLink(params["link_a"], m_rave->env);
    KinBody::LinkPtr linkB = GetCppLink(params["link_b"], m_rave->env);
    btVector3 pivotInA = toBtVector3(params["pivot_in_a"]) * METERS;
    btVector3 pivotInB = toBtVector3(params["pivot_in_b"]) * METERS;

    btRigidBody *rbA = findOrFail(m_rave->rave2bulletsim_links, linkA,
      (boost::format("link %s/%s not in bullet env") % linkA->GetParent()->GetName() % linkA->GetName()).str());
    btRigidBody *rbB = findOrFail(m_rave->rave2bulletsim_links, linkB,
      (boost::format("link %s/%s not in bullet env") % linkB->GetParent()->GetName() % linkB->GetName()).str());
    cnt = new btPoint2PointConstraint(*rbA, *rbB, pivotInA, pivotInB);

  } else if (type == "generic6dof") {
    KinBody::LinkPtr linkA = GetCppLink(params["link_a"], m_rave->env);
    KinBody::LinkPtr linkB = GetCppLink(params["link_b"], m_rave->env);
    btTransform frameInA = toBtTransform(params["frame_in_a"], METERS);
    btTransform frameInB = toBtTransform(params["frame_in_b"], METERS);
    bool useLinearReferenceFrameA = py::extract<bool>(params["use_linear_reference_frame_a"]);

    btRigidBody *rbA = findOrFail(m_rave->rave2bulletsim_links, linkA,
      (boost::format("link %s/%s not in bullet env") % linkA->GetParent()->GetName() % linkA->GetName()).str());
    btRigidBody *rbB = findOrFail(m_rave->rave2bulletsim_links, linkB,
      (boost::format("link %s/%s not in bullet env") % linkB->GetParent()->GetName() % linkB->GetName()).str());
    cnt = new btGeneric6DofConstraint(*rbA, *rbB, frameInA, frameInB, useLinearReferenceFrameA);

  } else {
    throw std::runtime_error((boost::format("constraint type %s not recognized") % type).str());
  }

  bool disable_collision_between_linked_bodies = false;
  if (params.has_key("disable_collision_between_linked_bodies")) {
    disable_collision_between_linked_bodies = py::extract<bool>(params["disable_collision_between_linked_bodies"]);
  }
  if (params.has_key("stop_erp")) {
    cout << "stop_erp" << py::extract<btScalar>(params["stop_erp"]) << endl;
    cnt->setParam(BT_CONSTRAINT_STOP_ERP, py::extract<btScalar>(params["stop_erp"]));
  }
  if (params.has_key("stop_cfm")) {
    cout << "stop_cfm" << py::extract<btScalar>(params["stop_cfm"]) << endl;
    cnt->setParam(BT_CONSTRAINT_STOP_CFM, py::extract<btScalar>(params["stop_cfm"]));
  }

  return AddConstraint(BulletConstraint::Ptr(new BulletConstraint(cnt, disable_collision_between_linked_bodies)));
}

void BulletEnvironment::RemoveConstraint(BulletConstraint::Ptr cnt) {
  m_env->removeConstraint(cnt);
}




static string makeRaveCylsXML(string name, btScalar radius, const vector<btScalar> &lengths) {
  stringstream xml;
  xml << "<Environment><KinBody name=\"" << name << "\">";
  for (int i = 0; i < lengths.size(); ++i) {
    btScalar len = lengths[i];
    xml << "<Body name=\"" << (boost::format("%s_%d") % name % i) << "\" type=\"static\"><Geom type=\"cylinder\">";
    xml << "<radius>" << radius << "</radius>";
    xml << "<height>" << len << "</height>";
    xml << "<RotationAxis>0 0 1 90</RotationAxis>";
    xml << "</Geom></Body>";
  }
  xml << "</KinBody></Environment>";
  return xml.str();
}

static vector<btRigidBody*> extractRigidBodies(const vector<RaveLinkObject::Ptr> &children) {
  vector<btRigidBody*> out(children.size());
  for (int i = 0; i < children.size(); ++i) {
    out[i] = children[i]->rigidBody.get();
  }
  return out;
}


CapsuleRope::CapsuleRope(BulletEnvironmentPtr env, const string& name, const vector<btVector3>& ctrlPoints, const CapsuleRopeParams& params) {
  init(env, name, ctrlPoints, params);
}

CapsuleRope::CapsuleRope(BulletEnvironmentPtr env, const string& name, py::object ctrlPoints, const CapsuleRopeParams& params) {
  vector<btVector3> v;
  fromNdarray2ToBtVecs(numpy.attr("asarray")(ctrlPoints), v);
  init(env, name, v, params);
}

void CapsuleRope::init(BulletEnvironmentPtr env, const string& name, const vector<btVector3>& ctrlPoints, const CapsuleRopeParams& params) {
  m_params = params;
  int nLinks = ctrlPoints.size()-1;
  vector<btTransform> transforms;
  vector<btScalar> lengths;
  CapsuleRope_createRopeTransforms(transforms,lengths,ctrlPoints);

  // cout << "got control points:\n";
  // for (int i = 0; i < ctrlPoints.size(); ++i) {
  //   cout << '\t' << ctrlPoints[i].x() << ' ' << ctrlPoints[i].y() << ' ' << ctrlPoints[i].z() << endl;
  // }

  env->GetRaveEnv()->LoadData(makeRaveCylsXML(name, m_params.radius, lengths));
  OpenRAVE::KinBodyPtr kinbody = env->GetRaveEnv()->GetKinBody(name);

  std::vector<BulletConstraint::Ptr> bulletJoints;
  for (int i=0; i < nLinks; i++) {
    btTransform trans = transforms[i]; trans.setOrigin(trans.getOrigin()*METERS);
    btScalar len = lengths[i] * METERS;
    float mass = 1.;

    CapsuleObject::Ptr tmp(new CapsuleObject(mass,m_params.radius*METERS,len,trans)); // only used for collision shape
    RaveLinkObject::Ptr link(new RaveLinkObject(env->GetRaveInstance(), kinbody->GetLinks()[i], mass, tmp->collisionShape, trans, false));
    link->rigidBody->setDamping(m_params.linDamping, m_params.angDamping);
    link->rigidBody->setFriction(BulletConfig::friction);
    //link->collisionShape->setMargin(0.04);
    m_children.push_back(link);

    kinbody->GetLinks()[i]->SetTransform(util::toRaveTransform(trans));

    if (i>0) {
      boost::shared_ptr<btPoint2PointConstraint> jointPtr(new btPoint2PointConstraint(*m_children[i-1]->rigidBody,*m_children[i]->rigidBody,btVector3(len/2,0,0),btVector3(-len/2,0,0)));
      jointPtr->setParam(BT_CONSTRAINT_STOP_ERP, m_params.linStopErp);
      bulletJoints.push_back(BulletConstraint::Ptr(new BulletConstraint(jointPtr, true)));

      boost::shared_ptr<btGeneric6DofSpringConstraint> springPtr = CapsuleRope_createBendConstraint(len,m_children[i-1]->rigidBody,m_children[i]->rigidBody,m_params.angDamping,m_params.angStiffness,m_params.angLimit);
      bulletJoints.push_back(BulletConstraint::Ptr(new BulletConstraint(springPtr, true)));
    }
  }

  m_obj.reset(new RaveObject(env->GetRaveInstance(), kinbody, m_children, bulletJoints, false));
  env->GetBulletEnv()->add(m_obj);

  m_children_rigidbodies = extractRigidBodies(m_children);
}

void CapsuleRope::UpdateRave() {
  const std::vector<KinBody::LinkPtr> &links = m_obj->body->GetLinks();
  assert(links.size() == m_children.size());
  for (int i = 0; i < m_children.size(); ++i) {
    links[i]->SetTransform(util::toRaveTransform(m_children[i]->rigidBody->getCenterOfMassTransform(), 1./METERS));
  }
}

void CapsuleRope::UpdateBullet() { throw std::runtime_error("CapsuleRope::UpdateBullet not supported"); }
void CapsuleRope::SetTransform(const btTransform&) { throw std::runtime_error("CapsuleRope::SetTransform not supported"); }
void CapsuleRope::SetLinearVelocity(const btVector3&){ throw std::runtime_error("CapsuleRope::SetLinearVelocity not supported"); }
void CapsuleRope::SetAngularVelocity(const btVector3&){ throw std::runtime_error("CapsuleRope::SetAngularVelocity not supported"); }

template<typename T, typename S>
static void scale(vector<T>& v, S w) {
  BOOST_FOREACH(T& x, v) {
    x *= w;
  }
}

std::vector<btVector3> CapsuleRope::GetNodes() {
  std::vector<btVector3> out = CapsuleRope_getNodes(m_children_rigidbodies);
  scale(out, 1.0f/METERS);
  return out;
}
std::vector<btVector3> CapsuleRope::GetControlPoints() {
  std::vector<btVector3> out = CapsuleRope_getControlPoints(m_children_rigidbodies);
  scale(out, 1.0f/METERS);
  return out;
}
vector<btMatrix3x3> CapsuleRope::GetRotations() {
  return CapsuleRope_getRotations(m_children_rigidbodies);
}
vector<float> CapsuleRope::GetHalfHeights() {
  std::vector<float> out = CapsuleRope_getHalfHeights(m_children_rigidbodies);
  scale(out, 1.0f/METERS);
  return out;
}

py::object CapsuleRope::py_GetNodes() { return toNdarray2(GetNodes()); }
py::object CapsuleRope::py_GetControlPoints() { return toNdarray2(GetControlPoints()); }
py::object CapsuleRope::py_GetRotations() { return toNdarray3(GetRotations()); }
py::object CapsuleRope::py_GetHalfHeights() { return toNdarray(GetHalfHeights()); }

} // namespace bs
