#pragma once

#include <Eigen/Dense>
using namespace Eigen;

#include "ophys_config.h"
#include "ophys_common.h"
using namespace ophys;

struct OptRope;

struct OptRopeState {

  struct StateAtTime {
    Vector7d manipDofs; // pos of manipulator 'gripper'
  
    MatrixX3d x; // positions of points (ith row is the position of the ith point)
    MatrixX3d vel; // velocities of points (ith row is the velocity of the ith point)

    VectorXd groundForce_f; // magnitudes of ground forces for each point
    VectorXd groundForce_c; // contact vars for ground forces

    MatrixX3d manipForce; // manipulator forces for each point
    VectorXd manipForce_c; // contact vars for manip forces

    MatrixX3d derived_accel; // not actually part of the state. filled in from cubic spline evaluation (only when expanded==true)
    Vector3d derived_manipPos;

    OptRope &m_opt;
    explicit StateAtTime(OptRope &opt, int N);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Conversion/utility methods

    int m_dim; int m_N;
    int dim() const { return m_dim; }

    bool isApprox(const StateAtTime &other) const;

    VectorXd toSubColumn() const {
      VectorXd col(dim());
      int pos = 0;
      col.segment<7>(pos) = manipDofs; pos += 7;
      for (int i = 0; i < m_N; ++i) {
        col.segment<3>(pos) = x.row(i).transpose(); pos += 3;
      }
      for (int i = 0; i < m_N; ++i) {
        col.segment<3>(pos) = vel.row(i).transpose(); pos += 3;
      }
      col.segment(pos, m_N) = groundForce_f; pos += m_N;
      col.segment(pos, m_N) = groundForce_c; pos += m_N;
      for (int i = 0; i < m_N; ++i) {
        col.segment<3>(pos) = manipForce.row(i).transpose(); pos += 3;
      }
      col.segment(pos, m_N) = manipForce_c; pos += m_N;
      assert(pos == dim());
      return col;
    }

    template<typename Derived>
    void initFromSubColumn(const DenseBase<Derived> &col) {
      assert(col.size() == dim());
      int pos = 0;
      manipDofs = col.template segment<7>(pos); pos += 7;
      for (int i = 0; i < m_N; ++i) {
        x.row(i) = (col.template segment<3>(pos)).transpose(); pos += 3;
      }
      for (int i = 0; i < m_N; ++i) {
        vel.row(i) = (col.template segment<3>(pos)).transpose(); pos += 3;
      }
      groundForce_f = col.segment(pos, m_N); pos += m_N;
      groundForce_c = col.segment(pos, m_N); pos += m_N;
      for (int i = 0; i < m_N; ++i) {
        manipForce.row(i) = (col.template segment<3>(pos)).transpose(); pos += 3;
      }
      manipForce_c = col.segment(pos, m_N); pos += m_N;
      assert(pos == dim());
    }

    string toString() const;
  };



  // The whole state is just a bunch of StateAtTimes over time
  EigVector<StateAtTime>::type atTime;

  int m_T, m_N;
  OptRope &m_opt;
  explicit OptRopeState(OptRope &opt, int T, int N)
    : atTime(T, StateAtTime(opt, N)),
      expanded(false),
      m_T(T), m_N(N),
      m_opt(opt)
  { }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int dim() const { return atTime[0].dim() * atTime.size(); }

  // Conversion/utility methods

  bool expanded;
  int getExpandedT(int interpPerTimestep) const { return (m_T - 1)*interpPerTimestep + 1; }
  // Gives a new OptRopeState with (m_T - 1)*interpPerTimestep + 1 points
  // using various interpolation schemes for the data
  OptRopeState expandByInterp(int interpPerTimestep) const;
  void fillExpansion(int interpPerTimestep, OptRopeState &out, const OptRopeState *mask=NULL, bool assumeOneMaskEntry=false) const;

  VectorXd toColumn() const {
    int subdim = atTime[0].dim();
    VectorXd col(atTime.size() * subdim);
    int pos = 0;
    for (int t = 0; t < atTime.size(); ++t) {
      col.segment(pos, subdim) = atTime[t].toSubColumn();
      pos += subdim;
    }
    assert(pos == col.size());
    return col;
  }

  template<typename Derived>
  void initFromColumn(const DenseBase<Derived> &col) {
    int subdim = atTime[0].dim();
    assert(col.size() == atTime.size() * subdim);
    int pos = 0;
    for (int t = 0; t < atTime.size(); ++t) {
      atTime[t].initFromSubColumn(col.segment(pos, subdim));
      pos += subdim;
    }
    assert(pos == col.size());
  }

  string toString() const {
    std::stringstream ss;
    for (int t = 0; t < atTime.size(); ++t) {
      ss << "t=" << t << (t % OPhysConfig::interpPerTimestep == 0 ? ": (*)" : ":") << "\n";
      ss << atTime[t].toString() << '\n';
    }
    return ss.str();
  }

  bool isApprox(const OptRopeState &other) const;
};
