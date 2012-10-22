#include "task_execution.h"
#include "rope_scenes.h"

#include "utils/logging.h"
#include "utils/vector_io.h"
#include "utils/conversions.h"

#include "lfd_python_wrapper.h"

const float table_dist_from_robot = 0.2;
const float table_width = 1, table_length = 1;
const float table_height = 0.7;
const float rope_len = 1.5;
const int rope_segments = 150;
static vector<btVector3> initTableCornersWorld() {
  vector<btVector3> v;
  v.push_back(METERS * btVector3(table_dist_from_robot, -table_width/2, table_height));
  v.push_back(METERS * btVector3(table_dist_from_robot, table_width/2, table_height));
  v.push_back(METERS * btVector3(table_dist_from_robot + table_length, table_width/2, table_height));
  v.push_back(METERS * btVector3(table_dist_from_robot + table_length, -table_width/2, table_height));
  return v;
}

struct LocalConfig : public Config {
  static string task;
  static string rope;
  static float pert;
  LocalConfig() : Config() { 
    params.push_back(new Parameter<string>("task", &task, "task name"));
    params.push_back(new Parameter<string>("rope", &rope, "rope control points file"));
    params.push_back(new Parameter<float>("pert", &pert, "rope perturbation variance"));
  }
};
string LocalConfig::task;
string LocalConfig::rope;
float LocalConfig::pert = 1.;

int main(int argc, char *argv[]) {
  GeneralConfig::scale = 10.;
  BulletConfig::maxSubSteps = 0;
  BulletConfig::gravity = btVector3(0, 0, -1);

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SceneConfig());
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);

  LoggingInit();
  Python_setup();

  // Initialize scene
  const btVector3 ropeStartPt = METERS * btVector3(table_dist_from_robot + table_length/2, 0, table_height + 0.1);
  const btVector3 ropeEndPt = METERS * btVector3(table_dist_from_robot + table_length/2, rope_len, table_height + 0.1);
  const btVector3 finalRopeStartPt = METERS * btVector3(table_dist_from_robot + table_length/2, 0, table_height + 0.1);
  const btVector3 finalRopeEndPt = METERS * btVector3(table_dist_from_robot + table_length/2, rope_len, table_height + 0.1);
  vector<btVector3> tableCorners = initTableCornersWorld();

  vector<btVector3> ropeCtlPts = toBulletVectors(floatMatFromFile(LocalConfig::rope)) * METERS;

  if (LocalConfig::pert != 0.) {
    try {
      lfd::CurvePerturbation cpert;
      ropeCtlPts = cpert.perturbCurve(ropeCtlPts, LocalConfig::pert * METERS);
    } catch (const py::error_already_set &e) {
      PyErr_Print();
      throw e;
    }
  }

  TableRopeScene scene(tableCorners, ropeCtlPts);
  scene.m_table->rigidBody->setFriction(0.9);
  scene.m_table->rigidBody->getCollisionShape()->setMargin(0.1);
  scene.startViewer();
  scene.stepFor(DT, 1);

  scene.viewer.frame();
  scene.setDrawing(true);

  lfd::TaskExecuter ex(scene);
  try {
    ex.run(LocalConfig::task);
  } catch (const py::error_already_set &e) {
    PyErr_Print();
    throw e;
  }

  scene.startFixedTimestepLoop(DT);
  return 0;
}
