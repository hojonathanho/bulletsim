#include "dynamics.h"
#include "task_execution.h"

using namespace lfd;

struct LocalConfig : public Config {
  static string task;
  static string rope;
  static float pert;
  static double trajSlow;
  static bool loadRopeFromDemo;
  LocalConfig() : Config() { 
    params.push_back(new Parameter<string>("task", &task, "task name"));
    params.push_back(new Parameter<float>("pert", &pert, "rope perturbation variance"));
    params.push_back(new Parameter<double>("trajSlow", &trajSlow, "slowdown factor for trajectory execution"));
  }
};
string LocalConfig::task;
float LocalConfig::pert = 1.;
double LocalConfig::trajSlow = 3.;

static void stepCallback(LFDRopeScene *s, SimpleTrajOptimizer::NominalTrajBuilder *trajbuilder, py::dict traj, int step) {
  trajbuilder->append(*s);
}

int main(int argc, char *argv[]) {
  LFDRopeScene s(argc, argv, LocalConfig());

  // execute trajectory, record SimpleTrajOptimizer::NominalTraj,
  // play that back and make sure it looks ok

  vector<btVector3> ropeCtlPts = loadRopeStateFromDemoCloud(LocalConfig::task, "00.00");
  s.resetScene(ropeCtlPts);
  util::setGlobalEnv(s.scene->env);

/*  s.scene->startViewer();
  s.scene->viewer.frame();
  s.scene->setDrawing(true);*/

  SimpleTrajOptimizer::NominalTrajBuilder trajbuilder;

  lfd::TaskExecuter ex(*s.scene);
  ex.setTrajExecSlowdown(LocalConfig::trajSlow);
  ex.setTrajStepCallback(boost::bind(stepCallback, &s, &trajbuilder, _1, _2));
  ex.setExecOneStep(true);
  try {
    ex.run(LocalConfig::task);
  } catch (const py::error_already_set &e) {
    Python_printError();
    throw;
  }

  SimpleTrajOptimizer::NominalTraj ntraj = trajbuilder.build();
  cout << "traj size " << ntraj.x.size() << ' ' << ntraj.u.size() << endl;
  cout << "state size " << ntraj.x[0].size() << " (" << ropeCtlPts.size() << ")" << '\n';
  cout << "control size " << ntraj.u[0].size() << '\n';

  LFDRopeScene s2(argc, argv, LocalConfig());
  s2.resetScene(ropeCtlPts);
  util::setGlobalEnv(s2.scene->env);
  s2.scene->startViewer();
  s2.scene->setDrawing(true);

  SimpleTrajOptimizer opt;
  opt.setDebuggingScene(s2.scene.get());

  opt.execNominalTraj(RopeRobotSystem::InitFrom(s2), ntraj);

  s2.scene->startFixedTimestepLoop(DT);

  return 0;
}
