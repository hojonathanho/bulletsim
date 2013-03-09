#include <boost/python.hpp>
#include "environment.h"
#include "openravesupport.h"
#include "logging.h"

using namespace Eigen;
using namespace OpenRAVE;
namespace py = boost::python;

static py::object openravepy, numpy;

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

class PyBulletEnvironment;
class PyBulletObject {
public:
  bool IsKinematic() { return m_obj->getIsKinematic(); }
  string GetName() { return m_obj->body->GetName(); }
  py::object GetKinBody() { return GetPyKinBody(m_obj->body); }

  py::object GetTransform() {
    btTransform t(m_obj->toRaveFrame(m_obj->children[0]->rigidBody->getCenterOfMassTransform()));
    btScalar mat[16];
    t.getOpenGLMatrix(mat);
    return toNdarray2(mat, 4, 4).attr("T");
  }

  void UpdateBullet() {
    m_obj->updateBullet();
  }

  void UpdateRave() {
    cout << "before env id: " << RaveGetEnvironmentId(m_obj->rave->env) << endl;
    m_obj->updateRave();
    cout << "after env id: " << RaveGetEnvironmentId(m_obj->rave->env) << endl;
  }

private:
  friend class PyBulletEnvironment;
  PyBulletObject(RaveObject::Ptr obj) : m_obj(obj) { }
  RaveObject::Ptr m_obj;
};
typedef boost::shared_ptr<PyBulletObject> PyBulletObjectPtr;

class PyBulletEnvironment {
public:
  PyBulletEnvironment(py::object py_rave_env, py::list dynamic_obj_names) {
    BulletInstance::Ptr bullet(new BulletInstance);
    m_env.reset(new Environment(bullet));
    m_rave.reset(new RaveInstance(GetCppEnv(py_rave_env)));
    LoadFromRaveExplicit(m_env, m_rave, toStrVec(dynamic_obj_names));
    m_env->bullet->setGravity(btVector3(0, 0, -9.8));
  }

//  PyBulletEnvironment(Environment::Ptr env, RaveInstance::Ptr rave) : m_env(env), m_rave(rave) {
//    m_env->bullet->setGravity(btVector3(0, 0, -9.8));
//  }

  ~PyBulletEnvironment() {
    cout << "py bullet env destroyed" << endl;
  }

  PyBulletObject GetObjectByName(const string &name) {
    return PyBulletObject(getObjectByName(m_env, m_rave, name));
  }

  PyBulletObject GetObjectFromKinBody(py::object py_kb) {
    if (openravepy.attr("RaveGetEnvironmentId")(py_kb.attr("GetEnv")()) != RaveGetEnvironmentId(m_rave->env)) {
      throw std::runtime_error("trying to get Bullet object for a KinBody that doesn't belong to this (OpenRAVE base) environment");
    }
    return PyBulletObject(getObjectByName(m_env, m_rave, GetCppKinBody(py_kb, m_rave->env)->GetName()));
  }

  py::object GetRaveEnv() {
    cout << "getting rave env with id " << RaveGetEnvironmentId(m_rave->env) << endl;
    return GetPyEnv(m_rave->env);
  }

  void SetGravity(py::list g) {
    m_env->bullet->setGravity(toBtVector3(g));
  }

  py::object GetGravity() {
    return toNdarray1(m_env->bullet->dynamicsWorld->getGravity().m_floats, 3);
  }

  void Step(float dt, int maxSubSteps, float fixedTimeStep) {
    m_env->step(dt, maxSubSteps, fixedTimeStep);
  }

private:
  Environment::Ptr m_env;
  RaveInstance::Ptr m_rave;
};
typedef boost::shared_ptr<PyBulletEnvironment> PyBulletEnvironmentPtr;

//PyBulletEnvironment PyLoadFromRave(py::object py_rave_env, py::list dynamic_obj_names) {
//  BulletInstance::Ptr bullet(new BulletInstance);
//  Environment::Ptr env(new Environment(bullet));
//  RaveInstance::Ptr rave(new RaveInstance(GetCppEnv(py_rave_env)));
//  LoadFromRaveExplicit(env, rave, toStrVec(dynamic_obj_names));
//  return PyBulletEnvironment(env, rave);
//}


BOOST_PYTHON_MODULE(cbulletsimpy) {
  LoggingInit();
  log4cplus::Logger::getRoot().setLogLevel(GeneralConfig::verbose);

  openravepy = py::import("openravepy");
  numpy = py::import("numpy");

  py::class_<PyBulletObject, PyBulletObjectPtr>("BulletObject", py::no_init)
    .def("IsKinematic", &PyBulletObject::IsKinematic)
    .def("GetName", &PyBulletObject::GetName)
    .def("GetKinBody", &PyBulletObject::GetKinBody, "get the KinBody in the OpenRAVE environment this object was created from")
    .def("GetTransform", &PyBulletObject::GetTransform)
    .def("UpdateBullet", &PyBulletObject::UpdateBullet, "set bullet object transform from the current transform in the OpenRAVE environment")
    .def("UpdateRave", &PyBulletObject::UpdateRave, "set the transform in the OpenRAVE environment from what it currently is in Bullet")
    ;

  py::class_<PyBulletEnvironment, PyBulletEnvironmentPtr>("BulletEnvironment", py::init<py::object, py::list>())
    .def("GetObjectByName", &PyBulletEnvironment::GetObjectByName, "get a BulletObject, given the OpenRAVE object name")
    .def("GetObjectFromKinBody", &PyBulletEnvironment::GetObjectFromKinBody, "")
    .def("GetRaveEnv", &PyBulletEnvironment::GetRaveEnv, "get the backing OpenRAVE environment")
    .def("SetGravity", &PyBulletEnvironment::SetGravity)
    .def("GetGravity", &PyBulletEnvironment::GetGravity)
    .def("Step", &PyBulletEnvironment::Step)
    ;

//  py::def("LoadFromRave", &PyLoadFromRave);
}
