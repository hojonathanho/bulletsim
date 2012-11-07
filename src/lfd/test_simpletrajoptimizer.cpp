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

static int execint_a = -1, execint_b = -1;
static void stepCallback(LFDRopeScene *s, SimpleTrajOptimizer::NominalTrajBuilder *trajbuilder, const lfd::Trajectory &traj, int step) {
  if (traj.lGrabTraj[step]) {
    if (execint_a == -1) execint_a = step;
  } else {
    if (execint_a != -1 && execint_b == -1) execint_b = step;
  }
  trajbuilder->append(*s);
}

int main(int argc, char *argv[]) {
  LFDRopeScene s(argc, argv, LocalConfig());

  // execute trajectory, record SimpleTrajOptimizer::NominalTraj,
  // play that back and make sure it looks ok

  vector<btVector3> ropeCtlPts = loadRopeStateFromDemoCloud(LocalConfig::task, "00.00");
  s.resetScene(ropeCtlPts);
  util::setGlobalEnv(s.scene->env);
/*
  s.scene->startViewer();
  s.scene->viewer.frame();
  s.scene->setDrawing(true);*/

  // execute controls from tpsrpm to get a nominal trajectory
  SimpleTrajOptimizer::NominalTrajBuilder trajbuilder;
  lfd::TaskExecuter ex(*s.scene);
  ex.init(LocalConfig::task);
  ex.setTrajExecSlowdown(LocalConfig::trajSlow);
  ex.setTrajStepCallback(boost::bind(stepCallback, &s, &trajbuilder, _1, _2));
  ex.setExecOneStep(true);
  try {
    ex.run(LocalConfig::task);
  } catch (const py::error_already_set &e) {
    Python_printError();
    throw;
  }

  const int ds = 3;
  trajbuilder.downsample(ds);
  int slice_start = execint_b/ds - 20, slice_end = execint_b/ds;
  SimpleTrajOptimizer::NominalTraj ntraj = trajbuilder.buildFromSlice(slice_start, slice_end);
  LOG_INFO("traj size " << ntraj.x.size() << ' ' << ntraj.u.size());
  LOG_INFO("state size " << ntraj.x[0].size() << " (" << ropeCtlPts.size() << ")");
  LOG_INFO("control size " << ntraj.u[0].size());
  LOG_INFO("total " << ntraj.x.size()*ntraj.x[0].size() + ntraj.u.size()*ntraj.u[0].size() << " variables");

  // start executing again, up to where we want to begin planning
  LFDRopeScene s2(argc, argv, LocalConfig());
  s2.resetScene(ropeCtlPts);
  util::setGlobalEnv(s2.scene->env);
  s2.scene->startViewer();
  s2.scene->setDrawing(true);

  lfd::TaskExecuter ex2(*s2.scene);
  ex2.init(LocalConfig::task);
  ex2.setPlotting(false);
  ex2.setTrajExecSlowdown(LocalConfig::trajSlow);
  ex2.setExecStepInterval(0, slice_start*ds);
  cout << "prep execution interval " << execint_a  << " to " << execint_b << endl;
  ex2.execRawTrajectory(ex.getLastExecutedTraj());

  SimpleTrajOptimizer opt(ropeCtlPts.size());
  opt.setDebuggingScene(s2.scene.get());
  opt.setInitSys(RopeRobotSystem::InitFrom(s2));
  opt.setInitTraj(ntraj);

/*  Scene dbgScene;
  opt.setDebuggingScene(&dbgScene);
  dbgScene.startViewer();
  dbgScene.setDrawing(true);
  opt.execNominalTraj(RopeRobotSystem::InitFrom(s2), ntraj);*/

  opt.run();

  s2.scene->startFixedTimestepLoop(DT);
//  dbgScene.startFixedTimestepLoop(DT);

  return 0;
}
