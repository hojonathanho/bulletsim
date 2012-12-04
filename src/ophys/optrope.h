#pragma once

#include <Eigen/Dense>
using namespace Eigen;

#include "optrope_state.h"
#include "simulation/openravesupport.h"

struct OptRope {
  const int m_N; // num particles
  const int m_T; // timesteps
  const double m_linkLen; // resting distance

  MatrixX3d m_initPos; // initial positions of the points (row(n) is the position of point n)
  Vector7d m_initManipDofs;

  struct CostCoeffs {
    double groundPenetration;
    double velUpdate;
    double contact;
    double forces;
    double linkLen;
    double goalPos;
    double manipSpeed;
    double damping;
  } m_coeffs;

  OptRope(int T, int N, double linkLen);

  void setInitPositions(const MatrixX3d &initPos);
  void setInitManipDofs(const Vector7d &initManipDofs);
  void calcBounds();

  void setCoeffs1();
  void setCoeffs2();

  bool m_useRobot;
  RaveRobotObject::Ptr m_robot;
  RobotManipulator::Ptr m_manip;
  void setRobot(RaveRobotObject::Ptr robot, RobotManipulator::Ptr manip);

  Vector3d calcManipPos(const Vector7d &dofs);

  template<typename Derived>
  OptRopeState toState(const DenseBase<Derived> &col) const {
    OptRopeState s(const_cast<OptRope *>(this), m_T, m_N);
    s.initFromColumn(col);
    return s;
  }

  static inline double infty() { return HUGE_VAL; }

  boost::shared_ptr<OptRopeState> m_lb, m_ub;
  VectorXd m_lbvec, m_ubvec;
  const OptRopeState &getLowerBound() const { return *m_lb; }
  const OptRopeState &getUpperBound() const { return *m_ub; }
  const VectorXd &getLowerBoundVec() const { return m_lbvec; }
  const VectorXd &getUpperBoundVec() const { return m_ubvec; }

  OptRopeState *createLowerBound();
  OptRopeState *createUpperBound();
  OptRopeState createInitState();

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

  OptRopeState m_dummyState;
  int getNumVariables() const {
    return m_dummyState.dim();
  }


  double cost_groundPenetration(const OptRopeState &es);
  double cost_velUpdate(const OptRopeState &es);
  double cost_contact(const OptRopeState &es);
  double cost_forces(const OptRopeState &es);
  double cost_linkLen(const OptRopeState &es);
  double cost_goalPos(const OptRopeState &es, const Vector3d &goal);
  double cost_manipSpeed(const OptRopeState &es);
  double cost_damping(const OptRopeState &es);

  double costfunc_on_expanded(const OptRopeState &expandedState);
  double costfunc(const OptRopeState &state);

  int m_costCalls;
  double costfunc_wrapper(const Eigen::Map<const VectorXd> &x);
  double nlopt_costWrapper(const vector<double> &x, vector<double> &grad);


  void writeState(const OptRopeState &state, const string &filename) const;
  OptRopeState readState(const string &filename) const;

#if 0
  template<typename VecType, typename Derived>
  void costGrad(DenseBase<Derived> &x0, VecType &out_grad) const {
    static const double eps = 1e-3;
    assert(out_grad.size() == x0.size());
    OptRopeState tmpstate(m_T, m_N);
    OptRopeState expansion(tmpstate.getExpandedT(OPhysConfig::interpPerTimestep), m_N);
    expansion.expanded = true;
    for (int i = 0; i < x0.size(); ++i) {
      double xorig = x0(i);
      x0[i] = xorig + eps;
      tmpstate.initFromColumn(x0); tmpstate.fillExpansion(OPhysConfig::interpPerTimestep, expansion); double b = costfunc_on_expanded(expansion);
      x0[i] = xorig - eps;
      tmpstate.initFromColumn(x0); tmpstate.fillExpansion(OPhysConfig::interpPerTimestep, expansion); double a = costfunc_on_expanded(expansion);
      x0[i] = xorig;
      out_grad[i] = (b - a) / (2*eps);
    }
  }

  template<typename VecType, typename Derived>
  VecType costGrad_orig(DenseBase<Derived> &x0) const {
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
#endif
};
