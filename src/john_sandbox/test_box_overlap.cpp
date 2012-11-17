#include <boost/python.hpp>
#include <openrave/openrave.h>
#include "simulation/simplescene.h"
#include "utils/config.h"
#include "simulation/util.h"
#include "simulation/bullet_io.h"
#include "sqp/collisions.h"
#include "sqp/utils_sqp.h"
#include "sqp/collisions.h"
#include "sqp/config_sqp.h"
using namespace OpenRAVE;
using namespace std;
using namespace util;

namespace py = boost::python;

int main(int argc, char* argv[]) {

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SQPConfig());
  parser.read(argc, argv);

  Py_Initialize();
  py::object main_module = py::import("__main__");
  py::object main_namespace = main_module.attr("__dict__");

  try {
    py::exec("import sys,os;"
      "execfile('/home/joschu/bulletsim/src/john_sandbox/test_box_overlap.py');", main_namespace);
  } catch (py::error_already_set err) {
    PyErr_Print();
    PyErr_Clear();
    throw;
  }
  EnvironmentBasePtr penv = OpenRAVE::RaveGetEnvironment(1);
  Scene scene(penv);
  LoadFromRave(scene.env, scene.rave);

  setupBulletForSQP(scene.env->bullet->dynamicsWorld);

  RaveObject::Ptr box0 = getObjectByName(scene.env, scene.rave, "box0");
  RaveObject::Ptr box1 = getObjectByName(scene.env, scene.rave, "box1");
  //  box0->children[0]->rigidBody->setContactProcessingThreshold(.1*METERS);
  //  box1->children[0]->rigidBody->setContactProcessingThreshold(.1*METERS);

  btDispatcher* dispatcher = scene.env->bullet->dynamicsWorld->getDispatcher();
  scene.env->bullet->dynamicsWorld->performDiscreteCollisionDetection();
  int numManifolds = dispatcher->getNumManifolds();
  for (int i = 0; i < numManifolds; ++i) {
    cout << "GOT A MANIFOLD!" << endl;
    btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);
    int numContacts = contactManifold->getNumContacts();
    btRigidBody* objA = static_cast<btRigidBody*> (contactManifold->getBody0());
    btRigidBody* objB = static_cast<btRigidBody*> (contactManifold->getBody1());
    for (int j = 0; j < numContacts; ++j) {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);
      cout << "GOT A POINT! " << pt.m_distance1 << " " << endl;
    }
  }

  printf("pairtest\n");

  scene.startViewer();
  PlotSpheresPtr spheres(new PlotSpheres());
  scene.env->add(spheres);
  while (true) {
    CollisionCollector cc;
    scene.env->bullet->dynamicsWorld->contactPairTest(box0->children[0]->rigidBody.get(),
                                                      box1->children[0]->rigidBody.get(), cc);
    vector<btVector3> centers;
    vector<btVector4> cols;
    vector<float> radii;
    BOOST_FOREACH(Collision& c, cc.m_collisions) {
      cout << "distance: " << c.m_distance + 2 * BulletConfig::linkPadding << " normal:  " << c.m_normal << " point: "
          << c.m_world0 << endl;
      centers.push_back(c.m_world0);
      centers.push_back(c.m_world1);
      cols.push_back(btVector4(1,0,0,1));
      cols.push_back(btVector4(1,0,0,1));
      radii.push_back(.05*METERS);
      radii.push_back(.05*METERS);
    }
    spheres->plot(toVec3Array(centers), toVec4Array(cols), radii);
    scene.step(.05);
    sleep(.05);
    scene.idle(true);
  }
}
