#include "task_execution.h"
#include "lfd_rope_common.h"
#include "lfd_python_wrapper.h"

#include "utils/vector_io.h"
#include "utils/conversions.h"

using namespace lfd;

struct LocalConfig : public Config {
  static string task;
  static string rope;
  static float pert;
  static double trajSlow;
  static bool loadRopeFromDemo;
  LocalConfig() : Config() { 
    params.push_back(new Parameter<string>("task", &task, "task name"));
    params.push_back(new Parameter<string>("rope", &rope, "rope control points file"));
    params.push_back(new Parameter<float>("pert", &pert, "rope perturbation variance"));
    params.push_back(new Parameter<double>("trajSlow", &trajSlow, "slowdown factor for trajectory execution"));
    params.push_back(new Parameter<bool>("loadRopeFromDemo", &loadRopeFromDemo, "load rope from demo observation"));
  }
};
string LocalConfig::task;
string LocalConfig::rope;
float LocalConfig::pert = 1.;
double LocalConfig::trajSlow = 1.;
bool LocalConfig::loadRopeFromDemo = false;

int main(int argc, char *argv[]) {
  LFDRopeScene s(argc, argv, LocalConfig());

  // Load rope
  vector<btVector3> ropeCtlPts;
  if (LocalConfig::loadRopeFromDemo) {
    ropeCtlPts = loadRopeStateFromDemoCloud(LocalConfig::task, "00.00");
  } else {
    ropeCtlPts = toBulletVectors(floatMatFromFile(LocalConfig::rope)) * METERS;
  }
  if (LocalConfig::pert != 0.) {
    try {
      lfd::CurvePerturbation cpert;
      ropeCtlPts = cpert.perturbCurve(ropeCtlPts, LocalConfig::pert * METERS);
    } catch (const py::error_already_set &e) {
      Python_printError();
      throw;
    }
  }
  s.resetScene(ropeCtlPts);
  util::setGlobalEnv(s.scene->env);

  s.scene->m_table->rigidBody->setFriction(0.9);
  s.scene->m_table->rigidBody->getCollisionShape()->setMargin(0.1);

  s.scene->startViewer();
  s.scene->viewer.frame();
  s.scene->setDrawing(true);

  lfd::TaskExecuter ex(*s.scene);
  ex.setTrajExecSlowdown(LocalConfig::trajSlow);
  try {
    ex.run(LocalConfig::task);
  } catch (const py::error_already_set &e) {
    Python_printError();
    throw;
  }

  s.scene->startFixedTimestepLoop(DT);

  return 0;
}
