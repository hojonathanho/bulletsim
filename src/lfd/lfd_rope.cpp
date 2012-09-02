#include "task_execution.h"
#include "rope_scenes.h"

#include "lfd_python_wrapper.h"
#include "utils/logging.h"

const float table_dist_from_robot = 0.3;
const float table_width = 1, table_length = 1;
const float table_height = 0.8;
const float rope_len = 1.5;
const int rope_segments = 100;
static vector<btVector3> initTableCornersWorld() {
  vector<btVector3> v;
  v.push_back(METERS * btVector3(table_dist_from_robot, -table_width/2, table_height));
  v.push_back(METERS * btVector3(table_dist_from_robot, table_width/2, table_height));
  v.push_back(METERS * btVector3(table_dist_from_robot + table_length, table_width/2, table_height));
  v.push_back(METERS * btVector3(table_dist_from_robot + table_length, -table_width/2, table_height));
  return v;
}

static vector<btVector3> initRopeControlPointsWorld() {
  const btVector3 startPt = METERS * btVector3(table_dist_from_robot + table_length/2, -rope_len/2, table_height + 0.1);
  const btVector3 endPt = METERS * btVector3(table_dist_from_robot + table_length/2, rope_len/2, table_height + 0.1);
  vector<btVector3> v;
  for (int i = 0; i < rope_segments; ++i) {
    float a = (float)i / rope_segments;
    v.push_back((1.-a)*startPt + a*endPt);
  }
  return v;
}

int main(int argc, char *argv[]) {
  GeneralConfig::scale = 100.;
  BulletConfig::maxSubSteps = 0;

  Parser parser;
  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SceneConfig());
  parser.read(argc, argv);

  LoggingInit();
  Python_setup();

  // Initialize scene
  TableRopeScene scene(initTableCornersWorld(), initRopeControlPointsWorld());
  scene.startViewer();
  scene.startLoop();

  lfd::TaskExecuter ex(scene);
  try {
    ex.run();
  } catch (const py::error_already_set &e) {
    PyErr_Print();
    throw e;
  }
  return 0;
}
