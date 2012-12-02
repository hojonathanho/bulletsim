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

  OptRope(const MatrixX3d &initPos, const Vector3d &initManipPos, int T, int N, double linkLen);

  void setCoeffs1();
  void setCoeffs2();

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

  OptRopeState createLowerBound() const;
  OptRopeState createUpperBound() const;
  OptRopeState createInitState() const;

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

  int getNumVariables() const {
    static const OptRopeState s(m_T, m_N);
    return s.dim();
  }

  double costfunc_on_expanded(const OptRopeState &expandedState) const;
  double costfunc(const OptRopeState &state) const;

  int m_costCalls;
  double costfunc_wrapper(const Eigen::Map<const VectorXd> &x);
  double nlopt_costWrapper(const vector<double> &x, vector<double> &grad) const;

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
