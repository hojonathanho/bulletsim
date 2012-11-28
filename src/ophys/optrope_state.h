#pragma once

#include <Eigen/Dense>
using namespace Eigen;


#include <boost/format.hpp>
#include <string>
#include <sstream>
using namespace std;

#include "ophys_config.h"
#include "ophys_common.h"
using namespace ophys;

#include "splines.h"

struct OptRopeState {

  struct StateAtTime {
    Vector3d manipPos; // pos of manipulator 'gripper'
  
    MatrixX3d x; // positions of points (ith row is the position of the ith point)
    MatrixX3d vel; // velocities of points (ith row is the velocity of the ith point)

    VectorXd groundForce_f; // magnitudes of ground forces for each point
    VectorXd groundForce_c; // contact vars for ground forces

    MatrixX3d manipForce; // manipulator forces for each point
    VectorXd manipForce_c; // contact vars for manip forces

    MatrixX3d derived_accel; // not actually part of the state. filled in from cubic spline evaluation (only when expanded==true)

    explicit StateAtTime(int N)
      : manipPos(Vector3d::Zero()),
        x(MatrixX3d::Zero(N, 3)),
        vel(MatrixX3d::Zero(N, 3)),
        groundForce_f(VectorXd::Zero(N)),
        groundForce_c(VectorXd::Zero(N)),
        manipForce(MatrixX3d::Zero(N, 3)),
        manipForce_c(VectorXd::Zero(N))
    {
      m_N = N;
      m_dim = manipPos.size()
            + x.size()
            + vel.size()
            + groundForce_f.size()
            + groundForce_c.size()
            + manipForce.size()
            + manipForce_c.size();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Conversion/utility methods

    int m_dim; int m_N;
    int dim() const { return m_dim; }

    VectorXd toSubColumn() const {
      VectorXd col(dim());
      int pos = 0;
      col.segment<3>(pos) = manipPos; pos += 3;
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

    void initFromSubColumn(const VectorXd &col) {
      assert(col.size() == dim());
      int pos = 0;
      manipPos = col.segment<3>(pos); pos += 3;
      for (int i = 0; i < m_N; ++i) {
        x.row(i) = (col.segment<3>(pos)).transpose(); pos += 3;
      }
      for (int i = 0; i < m_N; ++i) {
        vel.row(i) = (col.segment<3>(pos)).transpose(); pos += 3;
      }
      groundForce_f = col.segment(pos, m_N); pos += m_N;
      groundForce_c = col.segment(pos, m_N); pos += m_N;
      for (int i = 0; i < m_N; ++i) {
        manipForce.row(i) = (col.segment<3>(pos)).transpose(); pos += 3;
      }
      manipForce_c = col.segment(pos, m_N); pos += m_N;
      assert(pos == dim());
    }

    string toString() const {
      return (
        boost::format("> manipPos: %s\n> pos:\n%s\n> vel:\n%s\n> groundForce_f:%f\n> groundForce_c:%f\n> manipForce:\n%s\n> manipForce_c:%f\n")
          % manipPos.transpose()
          % x
          % vel
          % groundForce_f.transpose()
          % groundForce_c.transpose()
          % manipForce
          % manipForce_c.transpose()
      ).str();
    }
  };



  // The whole state is just a bunch of StateAtTimes over time
  EigVector<StateAtTime>::type atTime;

  explicit OptRopeState(int T, int N)
    : atTime(T, StateAtTime(N)),
      expanded(false)
  { }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int dim() const { return atTime[0].dim() * atTime.size(); }

  // Conversion/utility methods

  bool expanded;
  // Gives a new OptRopeState with (m_T - 1)*interpPerTimestep + 1 points
  // using various interpolation schemes for the data
  OptRopeState expandByInterp(int interpPerTimestep) const {
    const int origT = atTime.size(); const int N = atTime[0].m_N;
    const int newT = (origT - 1)*interpPerTimestep + 1;
    const VectorXd tv(VectorXd::LinSpaced(origT, 0, origT-1));
    const VectorXd fine_tv(VectorXd::LinSpaced(newT, 0, origT-1));

    // manipPos: cubic splines
    // TODO: use linear interpolation instead?
    MatrixX3d coarse_manipPos(origT, 3);
    MatrixX3d coarse_manipPos_d(origT, 3);
    for (int t = 0; t < origT; ++t) {
      coarse_manipPos.row(t) = atTime[t].manipPos;
      // catmull-rom tangents (TODO: add and use manip velocity instead?)
      // (one-sided finite differences at the endpoints)
      if (t > 0 && t < origT - 1) {
        coarse_manipPos_d.row(t) = (atTime[t+1].manipPos - atTime[t-1].manipPos) / 2.0;
      } else if (t == 0) {
        coarse_manipPos_d.row(t) = atTime[t+1].manipPos - atTime[t].manipPos;
      } else {
        coarse_manipPos_d.row(t) = atTime[t].manipPos - atTime[t-1].manipPos;
      }
    }
    MatrixXd fine_manipPos, fine_manipPos_d, fine_manipPos_d2, fine_manipPos_d3;
    splines::multi_hermite_cubic_spline_value(
      tv, coarse_manipPos, coarse_manipPos_d,
      fine_tv, fine_manipPos, fine_manipPos_d, fine_manipPos_d2, fine_manipPos_d3
    );

    // x, vel: cubic splines for x
    // (use vel as spline tangents for x. then fine_vel will be the interpolated tangents of fine_x)
    vector<MatrixXd> fine_x(N), fine_vel(N), fine_accel(N);
    MatrixXd fine_x_n_d3; // unused
    for (int n = 0; n < N; ++n) {
      MatrixX3d coarse_x_n(origT, 3), coarse_vel_n(origT, 3);
      for (int t = 0; t < origT; ++t) {
        coarse_x_n.row(t) = atTime[t].x.row(n);
        coarse_vel_n.row(t) = atTime[t].vel.row(n);
      }
      splines::multi_hermite_cubic_spline_value(
        tv, coarse_x_n, coarse_vel_n,
        fine_tv, fine_x[n], fine_vel[n], fine_accel[n], fine_x_n_d3
      );
    }

    // fill in the result
    OptRopeState out(newT, N);
    for (int t = 0; t < origT - 1; ++t) {
      int tA = t*interpPerTimestep;
      int tB = (t + 1)*interpPerTimestep;

      for (int s = tA; s < tB || (tB == newT - 1 && s <= tB); ++s) {
        out.atTime[s].manipPos = fine_manipPos.row(s).transpose();

        out.atTime[s].derived_accel.resize(N, 3);
        for (int n = 0; n < N; ++n) {
          out.atTime[s].x.row(n) = fine_x[n].row(s);
          out.atTime[s].vel.row(n) = fine_vel[n].row(s);
          out.atTime[s].derived_accel.row(n) = fine_accel[n].row(s);
        }

        const double a = (s - tA) / (double) interpPerTimestep;

        out.atTime[s].groundForce_f = (1.-a)*atTime[t].groundForce_f + a*atTime[t+1].groundForce_f;
        out.atTime[s].groundForce_c = atTime[t].groundForce_c;

        out.atTime[s].manipForce = (1.-a)*atTime[t].manipForce + a*atTime[t+1].manipForce;
        out.atTime[s].manipForce_c = atTime[t].manipForce_c;


                  // sanity check at knots
        // if (s == tA) {
        //   cout << "======================\n";
        //   cout << "sanity: manipPos" << out.atTime[s].manipPos.transpose() << " | " << atTime[t].manipPos.transpose() << endl;
        //   cout << "sanity: x" << out.atTime[s].x << " | " << atTime[t].x << endl;
        //   cout << "sanity: v" << out.atTime[s].vel << " | " << atTime[t].vel << endl;
        // }
        //  assert(out.atTime[s].manipPos == atTime[t].manipPos);

          //assert(out.atTime[s].x == atTime[t].x);
          //assert(out.atTime[s].vel == atTime[t].vel);
       // }
      }
    }
    out.expanded = true;
    return out;
  }

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
};
