#include "dynamics.h"
#include "task_execution.h"
#include "optimization_simple.h"

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


static void recordStateAndStopLoop(LFDRopeScene *s, vector<btVector3> *destRopeCtlPts, vector<double> *destManipDofs) {
  cout << "Recorded rope state!" << endl;
  s->scene->stopLoop();
  *destRopeCtlPts = s->scene->m_rope->getControlPoints();
  *destManipDofs = s->scene->pr2m->pr2Left->getDOFValues();
}

static void initScene(LFDRopeScene &s, bool first, vector<btVector3> &initRopeCtlPts, vector<double> &initManipDofs) {
  initRopeCtlPts.clear();
  initManipDofs.clear();

  const float rope_len = .6;
  const int rope_n_pts = 30;
  for (int i = 0 ; i < rope_n_pts; ++i) {
    initRopeCtlPts.push_back(METERS * btVector3(0.5, -rope_len/2. + (rope_len/(rope_n_pts-1.)*i), 0.8));
  }
  if (first) {
    s.resetScene(initRopeCtlPts);
    util::setGlobalEnv(s.scene->env);
  } else {
    s.scene->m_lMonitor->release();
    s.scene->resetRope(initRopeCtlPts);

    bool drawing = s.scene->drawingOn;
    s.scene->setDrawing(false);
    s.scene->stepFor(DT, 1);
    s.scene->setDrawing(drawing);
  }

  s.scene->pr2m->pr2Left->setGripperAngle(0.08);
  s.scene->pr2m->pr2Left->moveByIK(btTransform(btQuaternion(0, M_PI, M_PI/2.), initRopeCtlPts.back() + btVector3(0, 0, -0.02)*METERS));
  s.scene->m_lMonitor->grab();
  initManipDofs = s.scene->pr2m->pr2Left->getDOFValues();
}

static void initScene(LFDRopeScene &s, bool first) {
  vector<btVector3> initRopeCtlPts;
  vector<double> initManipDofs;
  initScene(s, first, initRopeCtlPts, initManipDofs);
}

int main(int argc, char *argv[]) {
  vector<btVector3> initRopeCtlPts;
  vector<double> initManipDofs;

  vector<btVector3> destRopeCtlPts;
  vector<double> destManipDofs;

  LFDRopeScene s(argc, argv, LocalConfig());
  initScene(s, true, initRopeCtlPts, initManipDofs);

  // let user record dest pose
  cout << "\n>>>>>> Scene has been set up. Manipulate into target state, and then press 'c' <<<<<" << endl;
  s.scene->addVoidKeyCallback('c', boost::bind(&recordStateAndStopLoop, &s, &destRopeCtlPts, &destManipDofs));
  s.scene->startViewer();
  s.scene->setDrawing(true);
  s.scene->startFixedTimestepLoop(DT); // returns when user types 'c'

  // build a nominal trajectory by moving in a straight line in joint space
  initScene(s, false);
  SimpleTrajOptimizer::NominalTrajBuilder trajbuilder;
  const float TRAJ_HORIZON = 15;
  vector<double> interDofs(initManipDofs.size());
  cout << initManipDofs.size() << ' ' << destManipDofs.size() << endl;
  for (int t = 0; t < TRAJ_HORIZON; ++t) {
    double a = t/(TRAJ_HORIZON-1.);
    for (int i = 0; i < interDofs.size(); ++i) {
      interDofs[i] = (1.-a)*initManipDofs[i] + a*destManipDofs[i];
    }
    s.scene->pr2m->pr2Left->setDOFValues(interDofs);
    for (int steps = 0; steps < 3; ++steps) {
      s.scene->step(DT);
    }
    //trajbuilder.append(s.scene->m_rope->getControlPoints(), interDofs);
    trajbuilder.append(s.scene->m_rope->getControlPoints(), initManipDofs); // NOTE: TESTING ZERO CONTROLS
    s.scene->draw();
  }

  // set up optimizer
  SimpleTrajOptimizer::NominalTraj ntraj = trajbuilder.build();
  SimpleTrajOptimizer opt(initRopeCtlPts.size());
  initScene(s, false);
  opt.setInitSys(RopeRobotSystem::InitFrom(s));
  opt.setInitTraj(ntraj);
  opt.setDesiredEndState(opt.toState(destRopeCtlPts));
  opt.setHorizon(TRAJ_HORIZON);

  // play back nominal trajectory for sanity
  opt.setDebuggingScene(s.scene.get());
  opt.debugJacobianCalc(true);
//  opt.execNominalTraj(RopeRobotSystem::InitFrom(s), ntraj);
//  opt.displayNominalTraj(ntraj);

//    s.scene->startFixedTimestepLoop(DT);

  cout << "Running optimization!" << endl;
  SimpleTrajOptimizer::NominalTraj nt = opt.oneStep();
  opt.displayNominalTraj(nt);

  opt.execNominalTraj(RopeRobotSystem::InitFrom(s), nt);

  return 0;
}
