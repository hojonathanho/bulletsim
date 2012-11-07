#include <boost/python.hpp>
#include <openrave/openrave.h>
#include "simulation/simplescene.h"
#include "utils/config.h"
#include "simulation/util.h"
#include "simulation/bullet_io.h"
#include "sqp/traj_costs.h"
using namespace OpenRAVE;
using namespace std;
using namespace util;

namespace py = boost::python;

int main(int argc, char* argv []) {

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.read(argc, argv);

  Py_Initialize();
  py::object main_module = py::import("__main__");
  py::object main_namespace = main_module.attr("__dict__");

  try {
    py::exec(
        "import sys,os;"
        "execfile('/home/joschu/bulletsim/src/john_sandbox/test_box_overlap.py');"
        , main_namespace);
  }
  catch (py::error_already_set err) {
    PyErr_Print();
    PyErr_Clear();
    throw;
  }
  EnvironmentBasePtr penv = OpenRAVE::RaveGetEnvironment(1);
  Scene scene(penv);
  LoadFromRave(scene.env, scene.rave);

  RaveObject::Ptr box0 = getObjectByName(scene.env, scene.rave, "box0");
  RaveObject::Ptr box1 = getObjectByName(scene.env, scene.rave, "box1");
  CollisionCollector cc;
  scene.env->bullet->dynamicsWorld->contactPairTest(box0->children[0]->rigidBody.get(), box1->children[0]->rigidBody.get(), cc);

  BOOST_FOREACH(Collision& c, cc.m_collisions) {
    cout << "distance: " << c.m_distance+2*BulletConfig::linkPadding <<   " normal:  " <<  c.m_normal << endl;;
  }

  scene.startViewer();
  scene.startLoop();

}
