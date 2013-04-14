#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "bulletsim_lite.h"
#include "logging.h"

namespace py = boost::python;

BOOST_PYTHON_MODULE(cbulletsimpy) {
  LoggingInit();
  log4cplus::Logger::getRoot().setLogLevel(GeneralConfig::verbose);

  bs::InitPython();

  py::class_<bs::BulletObject, bs::BulletObjectPtr>("BulletObject", py::no_init)
    .def("IsKinematic", &bs::BulletObject::IsKinematic)
    .def("GetName", &bs::BulletObject::GetName)
    .def("GetKinBody", &bs::BulletObject::py_GetKinBody, "get the KinBody in the OpenRAVE environment this object was created from")
    .def("GetTransform", &bs::BulletObject::py_GetTransform)
    .def("SetTransform", &bs::BulletObject::py_SetTransform)
    .def("SetLinearVelocity", &bs::BulletObject::py_SetLinearVelocity)
    .def("SetAngularVelocity", &bs::BulletObject::py_SetAngularVelocity)
    .def("UpdateBullet", &bs::BulletObject::UpdateBullet, "set bullet object transform from the current transform in the OpenRAVE environment")
    .def("UpdateRave", &bs::BulletObject::UpdateRave, "set the transform in the OpenRAVE environment from what it currently is in Bullet")
    ;
  py::class_<vector<bs::BulletObjectPtr> >("vector_BulletObject")
    .def(py::vector_indexing_suite<vector<bs::BulletObjectPtr>, true>());

  py::class_<bs::Collision, bs::CollisionPtr>("Collision", py::no_init)
    .add_property("linkA", &bs::Collision::py_linkA)
    .add_property("linkB", &bs::Collision::py_linkB)
    .add_property("ptA", &bs::Collision::py_ptA)
    .add_property("ptB", &bs::Collision::py_ptB)
    .add_property("normalB2A", &bs::Collision::py_normalB2A)
    .def_readonly("distance", &bs::Collision::distance)
    .def_readonly("weight", &bs::Collision::weight)
    .def("Flipped", &bs::Collision::Flipped)
    ;
  py::class_<vector<bs::CollisionPtr> >("vector_Collision")
    .def(py::vector_indexing_suite<vector<bs::CollisionPtr>, true>());

  py::class_<bs::BulletEnvironment, bs::BulletEnvironmentPtr>("BulletEnvironment", py::init<py::object, py::list>())
    .def("GetObjectByName", &bs::BulletEnvironment::GetObjectByName, "get a BulletObject, given the OpenRAVE object name")
    .def("GetObjectFromKinBody", &bs::BulletEnvironment::py_GetObjectFromKinBody, "")
    .def("GetObjects", &bs::BulletEnvironment::GetObjects, "get all objects")
    .def("GetDynamicObjects", &bs::BulletEnvironment::GetDynamicObjects, "get dynamic objects")
    .def("GetRaveEnv", &bs::BulletEnvironment::py_GetRaveEnv, "get the backing OpenRAVE environment")
    .def("SetGravity", &bs::BulletEnvironment::py_SetGravity)
    .def("GetGravity", &bs::BulletEnvironment::py_GetGravity)
    .def("Step", &bs::BulletEnvironment::Step)
    .def("DetectAllCollisions", &bs::BulletEnvironment::DetectAllCollisions)
    .def("ContactTest", &bs::BulletEnvironment::ContactTest)
    .def("SetContactDistance", &bs::BulletEnvironment::SetContactDistance)
    ;
}
