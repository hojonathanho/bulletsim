#include "task_execution.h"
#include "lfd_rope_common.h"
#include "lfd_python_wrapper.h"

#include "utils/vector_io.h"
#include "utils/conversions.h"
#include "utils/logging.h"

using namespace lfd;

struct LocalConfig : public Config {
  static string task;
  static string rope;
  static float pert;
  static double trajSlow;
  static bool loadRopeFromDemo;
  static float loadedRopeSegLen;
  static float loadedRopeScale;
  static float loadedRopeTranslateX;
  static float loadedRopeTranslateY;
  LocalConfig() : Config() { 
    params.push_back(new Parameter<string>("task", &task, "task name"));
    params.push_back(new Parameter<string>("rope", &rope, "rope control points file"));
    params.push_back(new Parameter<float>("pert", &pert, "rope perturbation variance"));
    params.push_back(new Parameter<double>("trajSlow", &trajSlow, "slowdown factor for trajectory execution"));
    params.push_back(new Parameter<bool>("loadRopeFromDemo", &loadRopeFromDemo, "load rope from demo observation"));
    params.push_back(new Parameter<float>("loadedRopeSegLen", &loadedRopeSegLen, "num control points for rope (if loadRopeFromDemo == true)"));
    params.push_back(new Parameter<float>("loadedRopeScale", &loadedRopeScale, "loaded rope scale (wrt centroid)"));
    params.push_back(new Parameter<float>("loadedRopeTranslateX", &loadedRopeTranslateX, "loaded rope translate (x direction)"));
    params.push_back(new Parameter<float>("loadedRopeTranslateY", &loadedRopeTranslateY, "loaded rope translate (y direction)"));
  }
};
string LocalConfig::task;
string LocalConfig::rope;
float LocalConfig::pert = 0;
double LocalConfig::trajSlow = 2.;
bool LocalConfig::loadRopeFromDemo = false;
float LocalConfig::loadedRopeSegLen = 0.025;
float LocalConfig::loadedRopeScale = 1.;
float LocalConfig::loadedRopeTranslateX = 0.;
float LocalConfig::loadedRopeTranslateY = 0.;


static void adjustPts(vector<btVector3> &pts) {
  btVector3 centroid(0, 0, 0);
  for (int i = 0; i < pts.size(); ++i) {
    centroid += pts[i];
  }
  centroid /= (btScalar) pts.size();

  for (int i = 0; i < pts.size(); ++i) {
    pts[i] = (pts[i] - centroid)*LocalConfig::loadedRopeScale + centroid;
  }

  for (int i = 0; i < pts.size(); ++i) {
    pts[i] += btVector3(LocalConfig::loadedRopeTranslateX, LocalConfig::loadedRopeTranslateY, 0.) * METERS;
  }
}

int main(int argc, char *argv[]) {
  LFDRopeScene s(argc, argv, LocalConfig());

  // Load rope
  vector<btVector3> ropeCtlPts;
  if (LocalConfig::loadRopeFromDemo) {
    try {
      ropeCtlPts = loadRopeStateFromDemoCloud(LocalConfig::task, "00.00", LocalConfig::loadedRopeSegLen);
      adjustPts(ropeCtlPts);
    } catch (const py::error_already_set &e) {
      Python_printError();
      throw;
    }
    LOG_INFO("Initialized rope with " << ropeCtlPts.size() << " control points");
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
