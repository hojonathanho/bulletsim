#include "dynamics.h"

#include "lfd_rope_common.h"

namespace lfd {


RopeRobotSystem::Ptr RopeRobotSystem::InitFrom(const LFDRopeScene &s) {
  RopeRobotSystem::Ptr sys(new RopeRobotSystem);
  sys->env = s.scene->env;
  sys->manip = s.scene->pr2m->pr2Left;
  sys->robot = s.scene->pr2m->pr2;
  sys->rope = s.scene->m_rope;
  sys->scene = NULL;
  sys->assertIntegrity();
  return sys;
}

RopeRobotSystem::Ptr RopeRobotSystem::fork() const {
  BulletInstance::Ptr fork_bullet(new BulletInstance);
  OSGInstance::Ptr fork_osg(new OSGInstance);
  Fork::Ptr fork(new Fork(env, fork_bullet, fork_osg));
  RaveRobotObject::Ptr fork_robot =
    boost::static_pointer_cast<RaveRobotObject>(fork->forkOf(robot));
  CapsuleRope::Ptr fork_rope =
    boost::static_pointer_cast<CapsuleRope>(fork->forkOf(rope));
  RaveRobotObject::Manipulator::Ptr fork_manip =
    fork_robot->getManipByIndex(manip->index);

  RopeRobotSystem::Ptr sys(new RopeRobotSystem);
  sys->env = fork->env;
  sys->manip = fork_manip;
  sys->robot = fork_robot;
  sys->rope = fork_rope;
  sys->scene = scene;
  sys->assertIntegrity();
  return sys;
}

void RopeRobotSystem::enableDrawing(Scene *s) {
  scene = s;
  enableDrawing();
}

void RopeRobotSystem::enableDrawing() {
  if (scene && !scene->osg->root->containsNode(env->osg->root.get())) {
    scene->osg->root->addChild(env->osg->root.get());
  }
}

void RopeRobotSystem::draw() {
  if (scene) {
    scene->draw();
  }
}

void RopeRobotSystem::disableDrawing() {
  if (scene) {
    scene->osg->root->removeChild(env->osg->root.get());
  }
  scene = NULL;
}

RopeRobotSystem::~RopeRobotSystem() {
  disableDrawing();
}

void RopeRobotSystem::assertIntegrity() {
  assert(env);
  assert(robot);
  assert(manip);
  assert(rope);
}

SimpleTrajOptimizer::SimpleTrajOptimizer() : dbgScene(NULL) {
}

struct SimpleTrajOptimizerImpl {
  static SimpleTrajOptimizer::State toState(const RopeState &pts) {
    SimpleTrajOptimizer::State s = Eigen::VectorXd::Zero(pts.size()*3);
    for (int i = 0; i < pts.size(); ++i) {
      s(3*i)   = pts[i].x();
      s(3*i+1) = pts[i].y();
      s(3*i+2) = pts[i].z();
    }
    return s;
  }

  static SimpleTrajOptimizer::State extractState(RopeRobotSystem::Ptr sys) {
    return toState(sys->rope->getControlPoints());
  }

  static SimpleTrajOptimizer::Control toControl(const btTransform &manipTrans0, const btTransform &manipTrans1) {
    btVector3 dx = manipTrans1.getOrigin() - manipTrans0.getOrigin();
    btScalar rx0, ry0, rz0; manipTrans0.getBasis().getEulerZYX(rz0, ry0, rx0);
    btScalar rx1, ry1, rz1; manipTrans1.getBasis().getEulerZYX(rz1, ry1, rx1);
    SimpleTrajOptimizer::Control u;
    u << dx.x(),  dx.y(), dx.z(),
         (rx1 - rx0), (ry1 - ry0), (rz1 - rz0);
    return u;
  }

  static void applyControlAndExec(RopeRobotSystem::Ptr sys, const SimpleTrajOptimizer::Control &u) {
    btTransform tr = sys->manip->getTransform();

    btTransform tr2; tr2.setIdentity();
    tr2.setOrigin(tr.getOrigin() + btVector3(u(0), u(1), u(2)));

    btScalar rx, ry, rz; tr.getBasis().getEulerZYX(rz, ry, rx);
    tr2.getBasis().setEulerZYX(rz + u(5), ry + u(4), rx + u(3));

    sys->manip->moveByIK(tr2);
    sys->env->step(DT, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);

    if (sys->scene) {
      cout << "gonna draw that shit" << endl;
      sys->enableDrawing();
      sys->scene->draw();
    }
  }

  static SimpleTrajOptimizer::NominalTraj execAndCalcControlJacobians(
    RopeRobotSystem::Ptr initSys,
    const SimpleTrajOptimizer::NominalTraj &traj,
    vector<Eigen::MatrixXd> &out_jacobians) {

    SimpleTrajOptimizer::NominalTraj outTraj;
    RopeRobotSystem::Ptr sys = initSys->fork();
    assert(traj.x.size() > 0);
    const int jrows = 3 * traj.x[0].size();
    const int jcols = 6; // 3 translational + 3 rotational dofs
    for (int i = 0; i < traj.u.size(); ++i) {
      // linearize around current u and save jacobian
      Eigen::MatrixXd j(jrows, jcols);
      for (int c = 0; c < jcols; ++c) {
        const double EPSILON = 0.01;

        RopeRobotSystem::Ptr tmpsys = sys->fork();
        SimpleTrajOptimizer::Control du = SimpleTrajOptimizer::Control::Zero(); du(c) = EPSILON;
        applyControlAndExec(tmpsys, du);
        SimpleTrajOptimizer::State s1 = extractState(tmpsys);
        assert(s1.rows() == jrows);

        tmpsys = sys->fork();
        du(c) = -EPSILON;
        applyControlAndExec(tmpsys, du);
        SimpleTrajOptimizer::State s2 = extractState(tmpsys);
        assert(s1.rows() == s2.rows());

        j.col(c) = (s1 - s2) / (2.*EPSILON);
      }
      out_jacobians.push_back(j);

      // apply nominal control and step
      outTraj.x.push_back(extractState(sys));
      outTraj.u.push_back(traj.u[i]);
      applyControlAndExec(sys, traj.u[i]);
    }

    outTraj.x.push_back(extractState(sys));
    assert(outTraj.x.size() == traj.x.size());
    assert(outTraj.u.size() == traj.u.size());
    return outTraj;
  }

  static SimpleTrajOptimizer::NominalTraj optimize(
    const SimpleTrajOptimizer::NominalTraj &nomTraj,
    const vector<Eigen::MatrixXd> &jacobians) {

  }
};


SimpleTrajOptimizer::NominalTraj SimpleTrajOptimizer::execNominalTraj(
  RopeRobotSystem::Ptr initSys,
  const SimpleTrajOptimizer::NominalTraj &traj
) {
  NominalTraj outTraj;
  RopeRobotSystem::Ptr sys = initSys->fork();
  sys->enableDrawing(dbgScene);
  assert(traj.x.size() > 0);
  for (int i = 0; i < traj.u.size(); ++i) {
    outTraj.x.push_back(SimpleTrajOptimizerImpl::extractState(sys));
    outTraj.u.push_back(traj.u[i]);
    SimpleTrajOptimizerImpl::applyControlAndExec(sys, traj.u[i]);
  }
  outTraj.x.push_back(SimpleTrajOptimizerImpl::extractState(sys));
  assert(outTraj.x.size() == traj.x.size());
  assert(outTraj.u.size() == traj.u.size());
  return outTraj;
}

SimpleTrajOptimizer::NominalTraj SimpleTrajOptimizer::toNominalTraj(
  const vector<RopeState> &ropeStates,
  const vector<btTransform> &manipPoses
) {
  assert(ropeStates.size() == manipPoses.size());
  NominalTraj n;
  for (int i = 0; i < ropeStates.size(); ++i) {
    n.x.push_back(SimpleTrajOptimizerImpl::toState(ropeStates[i]));
    if (i < manipPoses.size() - 1) {
      n.u.push_back(SimpleTrajOptimizerImpl::toControl(manipPoses[i], manipPoses[i+1]));
    }
  }
  return n;
}

void SimpleTrajOptimizer::NominalTrajBuilder::append(RopeRobotSystem::Ptr sys) {
  ropeStates.push_back(sys->rope->getControlPoints());
  manipPoses.push_back(sys->manip->getTransform());
}

void SimpleTrajOptimizer::NominalTrajBuilder::clear() {
  ropeStates.clear();
  manipPoses.clear();
}

SimpleTrajOptimizer::NominalTraj SimpleTrajOptimizer::NominalTrajBuilder::build() {
  return SimpleTrajOptimizer::toNominalTraj(ropeStates, manipPoses);
}

void SimpleTrajOptimizer::run() {
  NominalTraj traj = initTraj;

  // shooting method
  const int MAX_ITER = 100;
  for (int i = 0; i < MAX_ITER; ++i) {
    vector<Eigen::MatrixXd> jacobians;
    NominalTraj executed_traj = SimpleTrajOptimizerImpl::execAndCalcControlJacobians(initSys, traj, jacobians);
    traj = SimpleTrajOptimizerImpl::optimize(executed_traj, jacobians);
  }
}


} // namespace lfd
