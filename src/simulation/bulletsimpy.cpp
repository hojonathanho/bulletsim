#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "bulletsim_lite.h"
#include "logging.h"

namespace py = boost::python;

BOOST_PYTHON_MODULE(cbulletsimpy) {
  LoggingInit();
  log4cplus::Logger::getRoot().setLogLevel(GeneralConfig::verbose);

  bs::InitPython();

  py::register_exception_translator<std::exception>(&bs::TranslateStdException);

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

  py::class_<BulletConstraint, BulletConstraint::Ptr, boost::noncopyable>("BulletConstraint", py::no_init);

  py::class_<bs::SimulationParams>("SimulationParams")
    .def_readwrite("scale", &bs::SimulationParams::scale)
    .def_readwrite("gravity", &bs::SimulationParams::gravity)
    .def_readwrite("dt", &bs::SimulationParams::dt)
    .def_readwrite("maxSubSteps", &bs::SimulationParams::maxSubSteps)
    .def_readwrite("internalTimeStep", &bs::SimulationParams::internalTimeStep)
    .def_readwrite("friction", &bs::SimulationParams::friction)
    .def_readwrite("restitution", &bs::SimulationParams::restitution)
    .def_readwrite("margin", &bs::SimulationParams::margin)
    .def_readwrite("linkPadding", &bs::SimulationParams::linkPadding)
    ;

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
    .def("AddConstraint", &bs::BulletEnvironment::py_AddConstraint)
    .def("RemoveConstraint", &bs::BulletEnvironment::RemoveConstraint)
    ;

  py::class_<bs::CapsuleRopeParams>("CapsuleRopeParams")
    .def_readwrite("radius", &bs::CapsuleRopeParams::radius)
    .def_readwrite("angStiffness", &bs::CapsuleRopeParams::angStiffness)
    .def_readwrite("angDamping", &bs::CapsuleRopeParams::angDamping)
    .def_readwrite("linDamping", &bs::CapsuleRopeParams::linDamping)
    .def_readwrite("angLimit", &bs::CapsuleRopeParams::angLimit)
    .def_readwrite("linStopErp", &bs::CapsuleRopeParams::linStopErp)
    ;

  py::class_<bs::CapsuleRope, bs::CapsuleRopePtr, py::bases<bs::BulletObject> >("CapsuleRope", py::init<bs::BulletEnvironmentPtr, const string&, py::object, const bs::CapsuleRopeParams&>())
    .def("GetNodes", &bs::CapsuleRope::py_GetNodes)
    .def("GetControlPoints", &bs::CapsuleRope::py_GetControlPoints)
    .def("GetRotations", &bs::CapsuleRope::py_GetRotations)
    .def("GetHalfHeights", &bs::CapsuleRope::py_GetHalfHeights)
    ;

  py::scope().attr("sim_params") = bs::GetSimParams();
}
