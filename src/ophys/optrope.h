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

  double costfunc(const OptRopeState &state) const;

  int m_costCalls;
  double costfunc_wrapper(const Eigen::Map<const VectorXd> &x);
  double nlopt_costWrapper(const vector<double> &x, vector<double> &grad) const;

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

  template<typename VecType, typename Derived>
  VecType costGrad_openmp(const DenseBase<Derived> &x0) const {
    static const double eps = 1e-3;
    VecType grad(x0.size());

    #pragma openmp parallel for
    for (int i = 0; i < x0.size(); ++i) {
      VectorXd x(x0);
      OptRopeState tmpstate(m_T, m_N);
      double xorig = x[i];
      x[i] = xorig + eps;
      tmpstate.initFromColumn(x); double b = costfunc(tmpstate);
      x[i] = xorig - eps;
      tmpstate.initFromColumn(x); double a = costfunc(tmpstate);
      grad[i] = (b - a) / (2*eps);
    }
    return grad;
  }

};
