#include <boost/python.hpp>
#include "environment.h"
#include "openravesupport.h"

using namespace Eigen;
using namespace OpenRAVE;
namespace py = boost::python;

py::object openravepy, numpy;

/*
py::list toPyList(const IntVec& x) {
  py::list out;
  for (int i=0; i < x.size(); ++i) out.append(x[i]);
  return out;
}
*/

vector<string> toStrVec(py::list py_str_list) {
  int n = py::len(py_str_list);
  vector<string> out;
  for (int i = 0; i < n; ++i) {
    out.push_back(py::extract<string>(py_str_list[i]));
  }
  return out;
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
KinBodyPtr GetCppKinBody(py::object py_kb, EnvironmentBasePtr env) {
  int id = py::extract<int>(py_kb.attr("GetEnvironmentId")());
  return env->GetBodyFromEnvironmentId(id);
}
KinBody::LinkPtr GetCppLink(py::object py_link, EnvironmentBasePtr env) {
  KinBodyPtr parent = GetCppKinBody(py_link.attr("GetParent")(), env);
  int idx = py::extract<int>(py_link.attr("GetIndex")());
  return parent->GetLinks()[idx];
}

class PyBulletObject {
public:
  PyBulletObject(RaveObject::Ptr obj) : m_obj(obj) { }

  bool IsKinematic() { m_obj->getIsKinematic(); }

  OpenRAVE::Transform GetTransform() {
    return util::toRaveTransform(m_obj->children[0]->rigidBody->getCenterOfMassTransform(), 1./METERS);
  }

  void UpdateFromRave() {
    m_obj->updateBullet();
  }

private:
  RaveObject::Ptr m_obj;
};

class PyBulletEnvironment {
public:
  PyBulletEnvironment(Environment::Ptr env, RaveInstance::Ptr rave) : m_env(env), m_rave(rave) {
  }

  void SetGravity(const btVector3 &g) {
    m_env->bullet->setGravity(g);
  }

  py::object GetGravity() {
    return toNdarray1(m_env->bullet->dynamicsWorld->getGravity().m_floats, 3);
  }

  PyBulletObject GetObjectByName(const string &name) {
    return PyBulletObject(getObjectByName(m_env, m_rave, name));
  }

  void Step(float dt, int maxSubSteps, float fixedTimeStep) {
    m_env->step(dt, maxSubSteps, fixedTimeStep);
  }

private:
  Environment::Ptr m_env;
  RaveInstance::Ptr m_rave;
};

PyBulletEnvironment PyLoadFromRave(py::object py_rave_env, py::list dynamic_obj_names) {
  BulletInstance::Ptr bullet(new BulletInstance);
  Environment::Ptr env(new Environment(bullet));
  RaveInstance::Ptr rave(new RaveInstance(GetCppEnv(py_rave_env)));
  LoadFromRaveExplicit(env, rave, toStrVec(dynamic_obj_names));
  return PyBulletEnvironment(env, rave);
}


BOOST_PYTHON_MODULE(cbulletsimpy) {
  openravepy = py::import("openravepy");
  numpy = py::import("numpy");

  py::class_<PyBulletObject>("BulletObject", py::no_init)
    .def("IsKinematic", &PyBulletObject::IsKinematic)
    .def("GetTransform", &PyBulletObject::GetTransform)
    .def("UpdateFromRave", &PyBulletObject::UpdateFromRave)
    ;

  py::class_<PyBulletEnvironment>("BulletEnvironment", py::no_init)
    .def("Step", &PyBulletEnvironment::Step)
    .def("GetObjectByName", &PyBulletEnvironment::GetObjectByName)
    .def("SetGravity", &PyBulletEnvironment::SetGravity)
    .def("GetGravity", &PyBulletEnvironment::GetGravity)
    ;

  py::def("LoadFromRave", &PyLoadFromRave);
}
