#include "optimization_simple.h"

#include <gurobi_c++.h>
#include <boost/timer.hpp>
#include <boost/multi_array.hpp>

namespace lfd {

typedef boost::multi_array<GRBVar, 2> GRBVarMat;

static inline GRBQuadExpr square(const GRBLinExpr &e) { return e*e; }

struct SimpleTrajOptimizerImpl {
  friend class SimpleTrajOptimizer;
  SimpleTrajOptimizer *p;

  SimpleTrajOptimizer::NominalTraj initTraj;
  RopeRobotSystem::Ptr initSys;
  SimpleTrajOptimizer::State desiredEndState;
  int horizon;
  int stateDim;

  boost::shared_ptr<GRBEnv> grbEnv;
  boost::shared_ptr<GRBModel> grbModel;
  boost::shared_ptr<GRBVarMat> pvar_x, pvar_u;
  int nx, nu;

  Scene *dbgScene;
  bool debugJacobianCalc;


  SimpleTrajOptimizerImpl(SimpleTrajOptimizer *p_) : p(p_) {
    grbEnv.reset(new GRBEnv);
    grbModel.reset(new GRBModel(*grbEnv));
  }

  int getStateDim() const { return stateDim; }
  int getControlDim() const { return 7; }

  SimpleTrajOptimizer::State emptyState() const {
    return Eigen::VectorXd::Zero(getStateDim()*3);
  }

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

  static SimpleTrajOptimizer::Control toControl(const vector<double> &dofs0, const vector<double> &dofs1) {
    SimpleTrajOptimizer::Control u;
    assert(dofs0.size() == 7 && dofs1.size() == 7);
    for (int i = 0; i < 7; ++i) {
      u(i) = dofs1[i] - dofs0[i];
    }
    return u;
  }

  static void applyControlAndExec(RopeRobotSystem::Ptr sys, const SimpleTrajOptimizer::Control &u) {
    vector<double> dofs = sys->manip->getDOFValues();
    // 'gradually' apply u, since it may be large
    int usteps = std::min(10, 1 + (int)(u.maxCoeff() / 0.01));
    //cout << "usteps " << usteps << ' ' << u.maxCoeff() <<  endl;
    for (int us = 0; us < usteps; ++us) {
      for (int i = 0; i < 7; ++i) {
        dofs[i] += u(i)/((double)usteps);
      }
      sys->manip->setDOFValues(dofs);
      sys->env->step(DT, BulletConfig::maxSubSteps, BulletConfig::internalTimeStep);
      sys->draw();
    }
  }

  SimpleTrajOptimizer::NominalTraj execAndCalcControlJacobians(
    RopeRobotSystem::Ptr loc_initSys,
    const SimpleTrajOptimizer::NominalTraj &traj,
    vector<Eigen::MatrixXd> &out_jacobians) {
    /*
     * output J_i will be the linearization of the dynamics
     * around (x_i, u_i), so that f(x_i, u) ~= x_i + J_i*u
     */

    SimpleTrajOptimizer::NominalTraj outTraj;
    RopeRobotSystem::Ptr sys = loc_initSys->fork();
    assert(traj.x.size() > 0);
    const int jrows = getStateDim(); assert(jrows == traj.x[0].size());
    const int jcols = getControlDim(); assert(jcols == 7); // 7 dofs
    for (int i = 0; i < traj.u.size(); ++i) {
      // linearize around current u and save jacobian
      boost::timer timer;
      Eigen::MatrixXd j(jrows, jcols);
      for (int c = 0; c < jcols; ++c) {
        const double EPSILON = 0.05;
        RopeRobotSystem::Ptr tmpsys = sys->fork();
        RopeRobotSystem::ScopedLock::Ptr lock = tmpsys->lock();
        if (debugJacobianCalc) {
          tmpsys->enableDrawing(dbgScene);
        }
        SimpleTrajOptimizer::Control du = SimpleTrajOptimizer::Control::Zero(); du(c) = EPSILON;
        assert(du.size() == jcols);
        applyControlAndExec(tmpsys, du);
        SimpleTrajOptimizer::State s2 = extractState(tmpsys);
        lock.reset();

        tmpsys = sys->fork();
        if (debugJacobianCalc) {
          tmpsys->enableDrawing(dbgScene);
        }
        lock = tmpsys->lock();
        du(c) = -EPSILON;
        applyControlAndExec(tmpsys, du);
        SimpleTrajOptimizer::State s1 = extractState(tmpsys);
        lock.reset();

        j.col(c) = (s2 - s1) / (2.0 * EPSILON);
      }
      cout << " time to calc 1 jac : " << timer.elapsed() << endl;
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

  void opt_initVars(int nx, int nu) {
    // state variables x_i_j ( = jth component of state at time i)
    boost::format fmt("%c_%d_%d");
    pvar_x.reset(new GRBVarMat(boost::extents[nx][getStateDim()]));
    for (int i = 0; i < nx; ++i) {
      for (int j = 0; j < getStateDim(); ++j) {
        (*pvar_x)[i][j] = grbModel->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (fmt % 'x' % i % j).str());
      }
    }

    // control variables u_i_j
    pvar_u.reset(new GRBVarMat(boost::extents[nu][getControlDim()]));
    for (int i = 0; i < nu; ++i) {
      for (int j = 0; j < getControlDim(); ++j) {
        (*pvar_u)[i][j] = grbModel->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, (fmt % 'u' % i % j).str());
      }
    }
    grbModel->update();
  };

  void opt_addConstraints(
    const SimpleTrajOptimizer::NominalTraj &nomTraj,
    const vector<Eigen::MatrixXd> &jacobians
  ) {

    GRBVarMat &var_x = *pvar_x;
    GRBVarMat &var_u = *pvar_u;

    // robot dof constraints (sums of controls must be within dof limits)
    boost::format dof_cnt_fmt("doflim_%d_%d_%s");
    vector<double> initDofs = initSys->manip->getDOFValues();
    vector<double> upperLims, lowerLims;
    initSys->robot->robot->GetDOFLimits(lowerLims, upperLims, initSys->manip->manip->GetArmIndices());
    assert(initDofs.size() == getControlDim());
    assert(upperLims.size() == getControlDim());
    assert(lowerLims.size() == getControlDim());
    for (int i = 0; i < getControlDim(); ++i) {
      GRBLinExpr usum(initDofs[i]);
      for (int t = 0; t < nomTraj.u.size(); ++t) {
        usum += var_u[t][i];
        grbModel->addConstr(usum <= upperLims[i], (dof_cnt_fmt % t % i % "lt").str());
        grbModel->addConstr(usum >= lowerLims[i], (dof_cnt_fmt % t % i % "gt").str());
      }
    }

    // state trust region constraint (x_i in a box around nomTraj.x[i] forall i)
    // TODO: trust regions
    boost::format trust_fmt("trust_%s_%d_%d_%s");
    const float TRUST_X = 0.02 * METERS;
    for (int i = 0; i < nomTraj.x.size(); ++i) {
      for (int j = 0; j < getStateDim(); ++j) {
        GRBLinExpr dx = var_x[i][j] - nomTraj.x[i][j];
        grbModel->addConstr(dx <= TRUST_X, (trust_fmt % "x" % i % j % "lt").str());
        grbModel->addConstr(dx >= -TRUST_X, (trust_fmt % "x" % i % j % "gt").str());
      }
    }
    const float TRUST_U = 0.02;
    for (int t = 0; t < nomTraj.u.size(); ++t) {
      for (int j = 0; j < getControlDim(); ++j) {
        GRBLinExpr du = var_u[t][j] - nomTraj.u[t][j];
        grbModel->addConstr(du <= TRUST_U, (trust_fmt % "u" % t % j % "lt").str());
        grbModel->addConstr(du >= -TRUST_U, (trust_fmt % "u" % t % j % "gt").str());
      }
    }

    // rope initial condition constraint: x_0 == nom_x_0
    for (int i = 0; i < getStateDim(); ++i) {
      grbModel->addConstr(var_x[0][i] == nomTraj.x[0][i], (boost::format("x_init_%d") % i).str());
    }

    // rope dynamics constraint: x_(t+1) == x(t) + J_t*u_t
    boost::format rope_dyn_fmt("rope_dyn_%d_%d");
    for (int t = 0; t < nomTraj.x.size() - 1; ++t) {
      for (int i = 0; i < getStateDim(); ++i) {
        GRBLinExpr Ju_t_i(0); // ith component of J_t*u_t = (ith row of J_t)*u_t
        for (int j = 0; j < getControlDim(); ++j) {
          Ju_t_i += jacobians[t](i,j) * var_u[t][j];
        }
        grbModel->addQConstr(var_x[t+1][i] == var_x[t][i] + Ju_t_i, (rope_dyn_fmt % t % i).str());
      }
    }
  }

  void opt_setObjective(int nx, int nu) {
    GRBVarMat &var_x = *pvar_x;
    GRBVarMat &var_u = *pvar_u;

    GRBQuadExpr objective(0);

    // goal objective: ||x_H - X_H||^2
    for (int i = 0; i < getStateDim(); ++i) {
      GRBVar &x_H_i = var_x[nx-1][i];
      objective += square(x_H_i - desiredEndState(i));
    }

#if 0
    // rope dynamics objective: || (x_(t+1) - x_t) -  J_t*u_t ||^2
    for (int t = 0; t < nomTraj.x.size() - 1; ++t) {
      for (int i = 0; i < getStateDim(); ++i) {
        GRBLinExpr Ju_t_i(0); // ith component of J_t*u_t = (ith row of J_t)*u_t
        for (int j = 0; j < getControlDim(); ++j) {
          Ju_t_i += jacobians[t](i,j) * var_u[t][j];
        }
        objective += square((var_x[t+1][i] - var_x[t][i]) - Ju_t_i);
      }
    }
    // rope dynamics objective v2: || (x_(t+1) - nom_x_t) -  J_t*u_t ||^2
    for (int t = 0; t < nomTraj.x.size() - 1; ++t) {
      for (int i = 0; i < getStateDim(); ++i) {
        GRBLinExpr Ju_t_i(0); // ith component of J_t*u_t = (ith row of J_t)*u_t
        for (int j = 0; j < getControlDim(); ++j) {
          Ju_t_i += jacobians[t](i,j) * var_u[t][j];
        }
        objective += square((var_x[t+1][i] - nomTraj.x[t][i]) - Ju_t_i);
      }
    }
#endif

#if 0
    // rope physics objective: || dist(x_t_j, x_t_(j+1))^2 - D^2 ||^2
    for (int t = 0; t < nomTraj.x.size(); ++t) {
      for (int i = 0; i < getStateDim(); ++i) {
        square(var_x[t][i] - var_x[t+1][i]) +
        square(var_x[t][i+1] - var_x[t+1][i+1]) +
        square(var_x[t][i+2] - var_x[t+1][i+2])
        - ROPE_SEG_LEN*ROPE_SEG_LEN
      }
    }

    // rope physics objective: || z component of x_t_j - table_height ||^2
    for (int t = 0; t < nomTraj.x.size(); ++t) {
      for (int i = 2; i < getStateDim(); i += 3) {
        objective += square(var_x[t][i] - TABLE_HEIGHT);
      }
    }
#endif

    // control effort objective: sum_i(||u_i||^2)
    const double COEFF_CONTROL = 10;
    for (int t = 0; t < nu; ++t) {
      for (int i = 0; i < getControlDim(); ++i) {
        objective += COEFF_CONTROL * square(var_u[t][i]);
      }
    }

    grbModel->setObjective(objective);
    grbModel->update();
  };

  SimpleTrajOptimizer::NominalTraj opt_extractTrajFromGrb(int nx, int nu) {
    GRBVarMat &var_x = *pvar_x;
    GRBVarMat &var_u = *pvar_u;
    SimpleTrajOptimizer::NominalTraj outTraj;
    for (int t = 0; t < nx; ++t) {
      SimpleTrajOptimizer::State s = emptyState();
      for (int i = 0; i < getStateDim(); ++i) {
        s(i) = var_x[t][i].get(GRB_DoubleAttr_X);
      }
      outTraj.x.push_back(s);
    }
    for (int t = 0; t < nu; ++t) {
      SimpleTrajOptimizer::Control u = SimpleTrajOptimizer::Control::Zero();
      for (int i = 0; i < getControlDim(); ++i) {
        u(i) = var_u[t][i].get(GRB_DoubleAttr_X);
      }
      outTraj.u.push_back(u);
    }
    return outTraj;
  }

  SimpleTrajOptimizer::NominalTraj optimize(
    const SimpleTrajOptimizer::NominalTraj &nomTraj,
    const vector<Eigen::MatrixXd> &jacobians
      // jacobians must be generated from nomTraj for trust region constraints to make sense!
  ) {

    int nx = nomTraj.x.size(), nu = nomTraj.u.size();

    opt_initVars(nx, nu);
    opt_setObjective(nx, nu);

    opt_addConstraints(nomTraj, jacobians);

    try {
      grbModel->write("optimization.lp");
      grbModel->optimize();
    } catch (const GRBException &e) {
      LOG_ERROR("Gurobi exception (" << e.getErrorCode() << "): " << e.getMessage());
    }

    return opt_extractTrajFromGrb(nx, nu);
  }
};


SimpleTrajOptimizer::SimpleTrajOptimizer(int numCtlPts) : impl(new SimpleTrajOptimizerImpl(this)) {
  impl->dbgScene = NULL;
  impl->debugJacobianCalc = false;
  impl->stateDim = 3*numCtlPts;
}

void SimpleTrajOptimizer::setInitSys(RopeRobotSystem::Ptr sys) { impl->initSys = sys; }
void SimpleTrajOptimizer::setInitTraj(NominalTraj t) { impl->initTraj = t; }
void SimpleTrajOptimizer::setDesiredEndState(const State &s) { impl->desiredEndState = s; }
void SimpleTrajOptimizer::setHorizon(int h) { impl->horizon = h; }
void SimpleTrajOptimizer::setDebuggingScene(Scene *s) { impl->dbgScene = s; }
void SimpleTrajOptimizer::debugJacobianCalc(bool b) { impl->debugJacobianCalc = b; }

SimpleTrajOptimizer::NominalTraj SimpleTrajOptimizer::execNominalTraj(
  RopeRobotSystem::Ptr loc_initSys,
  const SimpleTrajOptimizer::NominalTraj &traj
) {
  NominalTraj outTraj;
  RopeRobotSystem::Ptr sys = loc_initSys->fork();
  RopeRobotSystem::ScopedLock::Ptr lock = sys->lock();

  sys->enableDrawing(impl->dbgScene);
  sys->draw();

  assert(traj.x.size() > 0);
  for (int i = 0; i < traj.u.size(); ++i) {
    outTraj.x.push_back(impl->extractState(sys));
    outTraj.u.push_back(traj.u[i]);
    impl->applyControlAndExec(sys, traj.u[i]);
    if(sys->scene) sys->scene->idle(true);
  }
  outTraj.x.push_back(impl->extractState(sys));
  assert(outTraj.x.size() == traj.x.size());
  assert(outTraj.u.size() == traj.u.size());
  return outTraj;
}

void SimpleTrajOptimizer::displayNominalTraj(const SimpleTrajOptimizer::NominalTraj &nt) {
  Scene *s = impl->dbgScene;
  vector<btVector3> points0, points1;
  for (int t = 0; t < nt.x.size(); ++t) {

    if (t < nt.u.size()) {
      cout << "current controls: ";
      for (int z = 0; z < nt.u[t].size(); ++z) {
        cout << nt.u[t][z] << ' ';
      }
      cout << endl;
    }

    points0.clear(); points1.clear();
    for (int i = 0; i < impl->getStateDim() - 3; i += 3) {
      points0.push_back(btVector3(nt.x[t][i  ], nt.x[t][i+1], nt.x[t][i+2]));
      points1.push_back(btVector3(nt.x[t][i+3], nt.x[t][i+4], nt.x[t][i+5]));
    }
    PlotLines::Ptr plot = util::drawLines(points0, points1, Eigen::Vector3f(1, 0, 0), 1, s->env);
//    s->idleFor(0.2);
    s->idle(true);
    s->env->remove(plot);
  }
}

SimpleTrajOptimizer::State SimpleTrajOptimizer::toState(const RopeState &rs) {
  return SimpleTrajOptimizerImpl::toState(rs);
}

SimpleTrajOptimizer::NominalTraj SimpleTrajOptimizer::toNominalTraj(
  const vector<RopeState> &ropeStates,
  const vector<vector<double> > &manipDofs
) {
  assert(ropeStates.size() == manipDofs.size());
  NominalTraj n;
  for (int i = 0; i < ropeStates.size(); ++i) {
    n.x.push_back(SimpleTrajOptimizerImpl::toState(ropeStates[i]));
    if (i < manipDofs.size() - 1) {
      n.u.push_back(SimpleTrajOptimizerImpl::toControl(manipDofs[i], manipDofs[i+1]));
    }
  }
  return n;
}

void SimpleTrajOptimizer::NominalTrajBuilder::append(const RopeState &ropeState, const vector<double> &manipDof) {
  ropeStates.push_back(ropeState);
  manipDofs.push_back(manipDof);
}

void SimpleTrajOptimizer::NominalTrajBuilder::append(RopeRobotSystem::Ptr sys) {
  append(sys->rope->getControlPoints(), sys->manip->getDOFValues());
}

void SimpleTrajOptimizer::NominalTrajBuilder::clear() {
  ropeStates.clear();
  manipDofs.clear();
}

SimpleTrajOptimizer::NominalTraj SimpleTrajOptimizer::NominalTrajBuilder::build() {
  return SimpleTrajOptimizer::toNominalTraj(ropeStates, manipDofs);
}

SimpleTrajOptimizer::NominalTraj SimpleTrajOptimizer::NominalTrajBuilder::buildFromSlice(int start, int end) {
  vector<RopeState> slicedRopeStates;
  vector<vector<double> > slicedManipDofs;
  for (int i = 0; i < ropeStates.size(); ++i) {
    if (i >= start && i <= end) {
      slicedRopeStates.push_back(ropeStates[i]);
      slicedManipDofs.push_back(manipDofs[i]);
    }
  }
  return SimpleTrajOptimizer::toNominalTraj(slicedRopeStates, slicedManipDofs);
}

void SimpleTrajOptimizer::NominalTrajBuilder::downsample(int x) {
  vector<RopeState> dsRopeStates;
  vector<vector<double> > dsManipDofs;
  for (int i = 0; i < ropeStates.size(); ++i) {
    if (i % x == 0) {
      dsRopeStates.push_back(ropeStates[i]);
      dsManipDofs.push_back(manipDofs[i]);
    }
  }
  ropeStates = dsRopeStates;
  manipDofs = dsManipDofs;
}

SimpleTrajOptimizer::NominalTraj SimpleTrajOptimizer::oneStep() {
  NominalTraj traj = impl->initTraj;
  boost::timer t;
  vector<Eigen::MatrixXd> jacobians;
  NominalTraj executed_traj = impl->execAndCalcControlJacobians(impl->initSys, traj, jacobians);
  cout << "time to exec and calc jacobians: " << t.elapsed() << endl;
  return impl->optimize(executed_traj, jacobians);
}

void SimpleTrajOptimizer::run() {
  NominalTraj traj = impl->initTraj;

  // shooting method
  const int MAX_ITER = 100;
  for (int i = 0; i < MAX_ITER; ++i) {
    boost::timer t;
    vector<Eigen::MatrixXd> jacobians;
    NominalTraj executed_traj = impl->execAndCalcControlJacobians(impl->initSys, traj, jacobians);
    cout << "time to exec and calc jacobians: " << t.elapsed() << endl;
    traj = impl->optimize(executed_traj, jacobians);
  }
}

} // namespace lfd
