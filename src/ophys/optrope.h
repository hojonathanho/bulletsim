#pragma once

#include <Eigen/Dense>
using namespace Eigen;

#include "optrope_state.h"

struct OptRope {
  const int m_N; // num particles
  const int m_T; // timesteps
  const double m_linkLen; // resting distance

  const MatrixX3d m_initPos; // initial positions of the points (row(n) is the position of point n)
  const Vector3d m_initManipPos;

  struct CostCoeffs {
    double groundPenetration;
    double velUpdate;
    double contact;
    double forces;
    double linkLen;
    double goalPos;
    double manipSpeed;
  } m_coeffs;
  const double GROUND_HEIGHT;

  OptRope(const MatrixX3d &initPos, const Vector3d &initManipPos, int T, int N, double linkLen)
    : m_N(N), m_T(T), m_initPos(initPos), m_initManipPos(initManipPos), m_linkLen(linkLen), GROUND_HEIGHT(0.),
      m_lb(createLowerBound()), m_ub(createUpperBound()),
      m_lbvec(createLowerBound().toColumn()), m_ubvec(createUpperBound().toColumn())
  {
    assert(m_N >= 1);
    assert(m_T >= 1);
    assert(initPos.rows() == m_N);
    setCoeffs1();
  }

  void setCoeffs1() {
    m_coeffs.groundPenetration = 1.;
    m_coeffs.velUpdate = 0.1;
    m_coeffs.contact = 0.1;
    m_coeffs.forces = 1;
    m_coeffs.linkLen = 0.1;
    m_coeffs.goalPos = 1;
    m_coeffs.manipSpeed = 0.001;
  }

  void setCoeffs2() {
    m_coeffs.groundPenetration = 1.;
    m_coeffs.velUpdate = 10.;
    m_coeffs.contact = 10;
    m_coeffs.forces = 1;
    m_coeffs.linkLen = 1;
    m_coeffs.goalPos = 1;
    m_coeffs.manipSpeed = 0.001;
  }


  template<typename Derived>
  OptRopeState toState(const DenseBase<Derived> &col) const {
    OptRopeState s(m_T, m_N);
    s.initFromColumn(col);
    return s;
  }

  static inline double infty() { return HUGE_VAL; }

  OptRopeState m_lb, m_ub;
  VectorXd m_lbvec, m_ubvec;
  const OptRopeState &getLowerBound() const { return m_lb; }
  const OptRopeState &getUpperBound() const { return m_ub; }
  const VectorXd &getLowerBoundVec() const { return m_lbvec; }
  const VectorXd &getUpperBoundVec() const { return m_ubvec; }

  OptRopeState createLowerBound() const {
    OptRopeState lb(m_T, m_N);
    for (int t = 0; t < m_T; ++t) {
      if (t == 0) lb.atTime[t].manipPos = m_initManipPos; else lb.atTime[t].manipPos.setConstant(-infty());
      if (t == 0) lb.atTime[t].x = m_initPos; else lb.atTime[t].x.setConstant(-infty());
      lb.atTime[t].vel.setConstant(t == 0 ? 0 : -infty());
      lb.atTime[t].groundForce_f.setZero();
      lb.atTime[t].groundForce_c.setConstant(0);
      lb.atTime[t].manipForce.setConstant(-infty());
      lb.atTime[t].manipForce_c.setConstant(0);
    }
    return lb;
  }

  OptRopeState createUpperBound() const {
    OptRopeState ub(m_T, m_N);
    for (int t = 0; t < m_T; ++t) {
      if (t == 0) ub.atTime[t].manipPos = m_initManipPos; else ub.atTime[t].manipPos.setConstant(infty());
      if (t == 0) ub.atTime[t].x = m_initPos; else ub.atTime[t].x.setConstant(infty());
      ub.atTime[t].vel.setConstant(t == 0 ? 0 : infty());
      ub.atTime[t].groundForce_f.setConstant(infty());
      ub.atTime[t].groundForce_c.setConstant(1);
      ub.atTime[t].manipForce.setConstant(infty());
      ub.atTime[t].manipForce_c.setConstant(1);
    }
    return ub;
  }

  template<typename VectorType>
  VectorType &clamp(VectorType &v) const {
    for (int i = 0; i < v.size(); ++i) {
      if (v[i] < m_lbvec[i]) {
        v[i] = m_lbvec[i];
      } else if (v[i] > m_ubvec[i]) {
        v[i] = m_ubvec[i];
      }
    }
    return v;
  }

  template<typename VectorType>
  VectorType &addNoiseClamped(VectorType &v, double mean, double stdev) const {
    addNoise(v, mean, stdev);
    clamp(v);
    return v;
  }

  OptRopeState createInitState() const {
    OptRopeState s(m_T, m_N);
    for (int t = 0; t < m_T; ++t) {
      s.atTime[t].manipPos = m_initManipPos;
      s.atTime[t].x = m_initPos;
      s.atTime[t].groundForce_c.setConstant(0.5);
      s.atTime[t].manipForce_c.setConstant(0.5);
      // everything else should be zero.
    }
    return s;
  }

  int getNumVariables() const {
    static const OptRopeState s(m_T, m_N);
    return s.dim();
  }

  ///////////// Cost function terms /////////////
  double cost_groundPenetration(const OptRopeState &es) const { // es is an expanded state (interpolated)
    // sum of squares of positive parts
    double cost = 0;
    for (int t = 0; t < es.atTime.size(); ++t) {
      for (int n = 0; n < m_N; ++n) {
        cost += square(max(0., GROUND_HEIGHT - es.atTime[t].x(n,2)));
      }
    }
    return cost;
  }

  double cost_velUpdate(const OptRopeState &es) const {
    assert(es.expanded);
    double cost = 0;
    for (int t = 0; t < es.atTime.size() - 1; ++t) {
      for (int n = 0; n < m_N; ++n) {
        // compute total force on particle n at time t
        Vector3d gravity(0, 0, OPhysConfig::gravity);
        Vector3d groundForce(0, 0, es.atTime[t].groundForce_c[n] * es.atTime[t].groundForce_f[n]);
        Vector3d manipForce = es.atTime[t].manipForce_c[n] * es.atTime[t].manipForce.row(n).transpose(); //(state.atTime[t].manipPos - pos[t].row(n).transpose()).normalized() * state.atTime[t].manipForce_f[n];
        Vector3d totalForce = gravity + groundForce + manipForce;
        Vector3d accel = es.atTime[t].derived_accel.row(n).transpose();
        cost += (accel - totalForce /* / mass */).squaredNorm();
      }
    }
    return cost;
  }

  double cost_contact(const OptRopeState &es) const {
    double cost = 0;
    // ground force: groundContact^2 * (ground non-violation)^2
    for (int t = 0; t < es.atTime.size(); ++t) {
      for (int n = 0; n < m_N; ++n) {
        // ground contact
        cost += es.atTime[t].groundForce_c[n] * square(max(0., es.atTime[t].x(n,2) - GROUND_HEIGHT));

        // manipulator contact
        cost += es.atTime[t].manipForce_c[n] * (es.atTime[t].manipPos - es.atTime[t].x.row(n).transpose()).squaredNorm();
      }
    }
    return cost;
  }

  // TODO: use f_total = sum(c*f), and get rid of f/c costs.
  double cost_forces(const OptRopeState &es) const {
    double cost = 0;
    for (int t = 0; t < es.atTime.size(); ++t) {
      for (int n = 0; n < m_N; ++n) {
        // ground force (force^2 / contact^2) + force^2
        //cost += square(es.atTime[t].groundForce_f[n]) / (1e-5 + square(es.atTime[t].groundForce_c[n]));
        cost += 1e-6*square(es.atTime[t].groundForce_f[n]);

        // manipulator force
        double manip_fsqnorm = es.atTime[t].manipForce.row(n).squaredNorm();
        //cost += manip_fsqnorm / (1e-5 + square(es.atTime[t].manipForce_c[n]));
        cost += 1e-6*manip_fsqnorm;
      }
    }
    return cost;
  }

  // rope segment distance violation cost
  double cost_linkLen(const OptRopeState &es) const {
    double cost = 0;
    for (int t = 0; t < es.atTime.size(); ++t) {
      for (int n = 0; n < m_N - 1; ++n) {
        double dist = (es.atTime[t].x.row(n) - es.atTime[t].x.row(n+1)).norm();
        cost += square(dist - m_linkLen);
      }
    }
    return cost;
  }

  // goal cost: point 0 should be at (0, 0, 1)
  double cost_goalPos(const OptRopeState &es) const {
    static const Vector3d desiredPos0(0, 1, 0);
    return (es.atTime.back().x.row(0).transpose() - desiredPos0).squaredNorm();
  }

  double cost_manipSpeed(const OptRopeState &es) const {
    double cost = 0;
    for (int t = 0; t < es.atTime.size() - 1; ++t) {
      cost += (es.atTime[t+1].manipPos - es.atTime[t].manipPos).squaredNorm();
    }
    return cost;
  }

  double costfunc(const OptRopeState &state) const {
    OptRopeState expandedState = state.expandByInterp(OPhysConfig::interpPerTimestep);

    double cost = 0;
    cost += m_coeffs.groundPenetration * cost_groundPenetration(expandedState);
    cost += m_coeffs.velUpdate * cost_velUpdate(expandedState);
    cost += m_coeffs.contact * cost_contact(expandedState);
    cost += m_coeffs.forces * cost_forces(expandedState);
    cost += m_coeffs.linkLen * cost_linkLen(expandedState);
    cost += m_coeffs.goalPos * cost_goalPos(expandedState);
    cost += m_coeffs.manipSpeed * cost_manipSpeed(expandedState);
    return cost;
  }

  double costfunc_wrapper(const Eigen::Map<const VectorXd> &x) {
    OptRopeState state(m_T, m_N);
    state.initFromColumn(x);
    return costfunc(state);
  }

  // OptRopeState solve() {
  //   return solve(genInitState());
  // }

  // State solve(const State &init) {
  //   return solve(init.toColumn());
  // }

  // State solve(const VectorXd &init) {
  //   cout << "initial vals: " << init.transpose() << endl;
  //   cout << "lb: " << genLowerBound().toColumn().transpose() << endl;
  //   cout << "ub: " << genUpperBound().toColumn().transpose() << endl;
  //   SmartPtr<TNLP> prob = new BoxConstrainedOptProblem(
  //     getNumVariables(),
  //     genLowerBound().toColumn(),
  //     genUpperBound().toColumn(),
  //     init,
  //     boost::bind(&OptRope::costfunc_wrapper, this, _1)
  //   );

  //   SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
  //   app->Options()->SetNumericValue("tol", 1e-3);
  //   app->Options()->SetStringValue("mu_strategy", "adaptive");
  //   app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  //   app->Options()->SetIntegerValue("print_level", 5);
  //   app->Options()->SetIntegerValue("max_iter", 10000);

  //   ApplicationReturnStatus status;
  //   status = app->Initialize();
  //   if (status != Solve_Succeeded) {
  //     std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
  //     throw std::runtime_error((boost::format("ipopt initialization error: %d") % status).str());
  //   }
  //   status = app->OptimizeTNLP(prob);
  //   if (status == Solve_Succeeded) {
  //     std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
  //   } else {
  //     std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
  //   }

  //   const BoxConstrainedOptProblem::Solution &soln = ((BoxConstrainedOptProblem*) GetRawPtr(prob))->solution();
  //   State s(m_T, m_N);
  //   s.initFromColumn(soln.x);
  //   return s;
  // }

  template<typename VecType, typename Derived>
  VecType costGrad(DenseBase<Derived> &x0) const {
    static const double eps = 1e-3;
    VecType grad(x0.size());
    OptRopeState tmpstate(m_T, m_N);
    for (int i = 0; i < x0.size(); ++i) {
      double xorig = x0(i);
      x0[i] = xorig + eps;
      tmpstate.initFromColumn(x0); double b = costfunc(tmpstate);
      x0[i] = xorig - eps;
      tmpstate.initFromColumn(x0); double a = costfunc(tmpstate);
      x0[i] = xorig;
      grad[i] = (b - a) / (2*eps);
    }
    return grad;
  }

  double nlopt_costWrapper(const vector<double> &x, vector<double> &grad) const {
    //VectorXd ex = toEigVec(x);
    Eigen::Map<VectorXd> ex(const_cast<double*>(x.data()), x.size(), 1);
    OptRopeState state(m_T, m_N); state.initFromColumn(ex);
    if (!grad.empty()) {
      vector<double> g = costGrad<vector<double> >(ex);
      assert(g.size() == grad.size());
      grad = g;
    }
    return costfunc(state);
  }

};