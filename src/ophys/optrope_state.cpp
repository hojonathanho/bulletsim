#include "optrope_state.h"
#include "optrope.h"
#include "splines.h"

#include <boost/format.hpp>
#include <string>
#include <sstream>
using namespace std;

OptRopeState::StateAtTime::StateAtTime(OptRope &opt, int N)
  : manipDofs(Vector7d::Zero()),
    x(MatrixX3d::Zero(N, 3)),
    vel(MatrixX3d::Zero(N, 3)),
    groundForce_f(VectorXd::Zero(N)),
    groundForce_c(VectorXd::Zero(N)),
    manipForce(MatrixX3d::Zero(N, 3)),
    manipForce_c(VectorXd::Zero(N)),
    m_opt(opt)
{
  m_N = N;
  m_dim = manipDofs.size()
        + x.size()
        + vel.size()
        + groundForce_f.size()
        + groundForce_c.size()
        + manipForce.size()
        + manipForce_c.size();
}

string OptRopeState::StateAtTime::toString() const {
  return (
    boost::format("> manipDofs: %s\n> pos:\n%s\n> vel:\n%s\n> groundForce_f:%f\n> groundForce_c:%f\n> manipForce:\n%s\n> manipForce_c:%f\n")
      % manipDofs.transpose()
      % x
      % vel
      % groundForce_f.transpose()
      % groundForce_c.transpose()
      % manipForce
      % manipForce_c.transpose()
  ).str();
}

OptRopeState OptRopeState::expandByInterp(int interpPerTimestep) const {
  const int origT = atTime.size(); const int N = atTime[0].m_N;
  const int newT = getExpandedT(interpPerTimestep);
  const ArrayXd tv(ArrayXd::LinSpaced(origT, 0, origT-1));
  const ArrayXd fine_tv(ArrayXd::LinSpaced(newT, 0, origT-1));

  // manipDofs: cubic splines
  // TODO: use linear interpolation instead?
  // MatrixX7d coarse_manipDofs(origT, 7);
  // MatrixX7d coarse_manipDofs_d(origT, 7);
  // for (int t = 0; t < origT; ++t) {
  //   coarse_manipDofs.row(t) = atTime[t].manipDofs.transpose();
  //   // catmull-rom tangents (TODO: add and use manip velocity instead?)
  //   // (one-sided finite differences at the endpoints)
  //   if (t > 0 && t < origT - 1) {
  //     coarse_manipDofs_d.row(t) = ((atTime[t+1].manipDofs - atTime[t-1].manipDofs) / 2.0).transpose();
  //   } else if (t == 0) {
  //     coarse_manipDofs_d.row(t) = (atTime[t+1].manipDofs - atTime[t].manipDofs).transpose();
  //   } else {
  //     coarse_manipDofs_d.row(t) = (atTime[t].manipDofs - atTime[t-1].manipDofs).transpose();
  //   }
  // }
  // MatrixXd fine_manipDofs, fine_manipDofs_d, fine_manipDofs_d2, fine_manipDofs_d3;
  // splines::multi_hermite_cubic_spline_value(
  //   tv, coarse_manipDofs, coarse_manipDofs_d,
  //   fine_tv, fine_manipDofs, fine_manipDofs_d, fine_manipDofs_d2, fine_manipDofs_d3
  // );


  // x, vel: cubic splines for x
  // (use vel as spline tangents for x. then fine_vel will be the interpolated tangents of fine_x)
  // TODO: pack everything into one matrix, and call spline interpolation once
  vector<MatrixXd> fine_x(N), fine_vel(N), fine_accel(N);
  MatrixXd fine_x_n_d3; // unused
  MatrixX3d coarse_x_n(origT, 3), coarse_vel_n(origT, 3);
  for (int n = 0; n < N; ++n) {
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
  OptRopeState out(m_opt, newT, N);
  for (int t = 0; t < origT - 1; ++t) {
    int tA = t*interpPerTimestep;
    int tB = (t + 1)*interpPerTimestep;

    for (int s = tA; s < tB || (tB == newT - 1 && s <= tB); ++s) {
      const double a = (s - tA) / (double) interpPerTimestep;

      // out.atTime[s].manipDofs = fine_manipDofs.row(s).transpose();
      out.atTime[s].manipDofs = (1.-a)*atTime[t].manipDofs + a*atTime[t+1].manipDofs;
      out.atTime[s].derived_manipPos = m_opt.calcManipPos(out.atTime[s].manipDofs); // really slow

      out.atTime[s].derived_accel.resize(N, 3);
      for (int n = 0; n < N; ++n) {
        out.atTime[s].x.row(n) = fine_x[n].row(s);
        out.atTime[s].vel.row(n) = fine_vel[n].row(s);
        out.atTime[s].derived_accel.row(n) = fine_accel[n].row(s);
      }


      out.atTime[s].groundForce_f = (1.-a)*atTime[t].groundForce_f + a*atTime[t+1].groundForce_f;
      out.atTime[s].groundForce_c = atTime[t].groundForce_c;

      out.atTime[s].manipForce = (1.-a)*atTime[t].manipForce + a*atTime[t+1].manipForce;
      out.atTime[s].manipForce_c = atTime[t].manipForce_c;


      // sanity check at knots
      // if (s == tA) {
      //   cout << "======================\n";
      //   cout << "sanity: manipDofs" << out.atTime[s].manipDofs.transpose() << " | " << atTime[t].manipDofs.transpose() << endl;
      //   cout << "sanity: x" << out.atTime[s].x << " | " << atTime[t].x << endl;
      //   cout << "sanity: v" << out.atTime[s].vel << " | " << atTime[t].vel << endl;
      // }
      //  assert(out.atTime[s].manipDofs == atTime[t].manipDofs);

        //assert(out.atTime[s].x == atTime[t].x);
        //assert(out.atTime[s].vel == atTime[t].vel);
     // }
    }
  }
  out.expanded = true;
  return out;
}

void OptRopeState::fillExpansion(int interpPerTimestep, OptRopeState &out, const OptRopeState *mask, bool assumeOneMaskEntry) const {
  const int origT = atTime.size(); const int N = atTime[0].m_N;
  const int newT = getExpandedT(interpPerTimestep);

  // fill in the result
  if (mask) {
    assert(mask->m_T == origT && mask->m_N == N && !mask->expanded);
  } else {
    assumeOneMaskEntry = false;
  }
  assert(out.m_T == newT && out.m_N == N && out.expanded);

  out.expanded = true;
  for (int t = 0; t < origT - 1; ++t) {

    bool calc_manipDofs = !mask || !mask->atTime[t].manipDofs.isZero() || !mask->atTime[t+1].manipDofs.isZero();
    bool calc_pos = !mask || !mask->atTime[t].x.isZero() || !mask->atTime[t+1].x.isZero() || !mask->atTime[t].vel.isZero() || !mask->atTime[t+1].vel.isZero();
    bool calc_groundForce = !mask || !mask->atTime[t].groundForce_f.isZero() || !mask->atTime[t+1].groundForce_f.isZero();
    bool calc_manipForce = !mask || !mask->atTime[t].manipForce.isZero() || !mask->atTime[t+1].manipForce.isZero();

    int tA = t*interpPerTimestep;
    int tB = (t + 1)*interpPerTimestep;

    Vector3d p1, p2;
    if (calc_manipDofs) {
      p1 = m_opt.calcManipPos(out.atTime[t].manipDofs);
      p2 = m_opt.calcManipPos(out.atTime[t+1].manipDofs);
    }

    for (int s = tA; s < tB || (tB == newT - 1 && s <= tB); ++s) {
      const double frac_s = s*(origT-1.)/(newT-1.);
      const double a = (s - tA) / (double) interpPerTimestep;
      // manipDofs
      // catmull-rom tangents
      // (one-sided finite differences at the endpoints)
      // Vector7d manipDofs_d1, manipDofs_d2;
      // if (t > 0 && t < origT - 2) {
      //   manipDofs_d1 = (atTime[t+1].manipDofs - atTime[t-1].manipDofs)/2.0;
      //   manipDofs_d2 = (atTime[t+2].manipDofs - atTime[t].manipDofs)/2.0;
      // } else if (t == 0) {
      //   manipDofs_d1 = atTime[t+1].manipDofs - atTime[t].manipDofs;
      //   manipDofs_d2 = (atTime[t+2].manipDofs - atTime[t].manipDofs)/2.0;
      // } else if (t == origT - 2) {
      //   manipDofs_d1 = (atTime[t+1].manipDofs - atTime[t-1].manipDofs)/2.0;
      //   manipDofs_d2 = atTime[t+1].manipDofs - atTime[t].manipDofs;
      // } else {
      //   assert(false);
      // }
      // splines::hermite_cubic_spline_value_single0(
      //   (double) t, (double) t+1,
      //   atTime[t].manipDofs, atTime[t+1].manipDofs,
      //   manipDofs_d1, manipDofs_d2,
      //   frac_s, out.atTime[s].manipDofs
      // );
      if (calc_manipDofs) {
        out.atTime[s].manipDofs = (1.-a)*atTime[t].manipDofs + a*atTime[t+1].manipDofs;
        // cheating...
        //out.atTime[s].derived_manipPos = m_opt.calcManipPos(out.atTime[s].manipDofs);
        out.atTime[s].derived_manipPos = (1.-a)*p1 + a*p2;
        if (assumeOneMaskEntry) return;
      }

      // pos, vel, accel
      if (calc_pos) {
        out.atTime[s].derived_accel.resize(N, 3);
        Vector3d x, vel, accel;
        for (int n = 0; n < N; ++n) {
          splines::hermite_cubic_spline_value_single2(
            (double) t, (double) t+1,
            atTime[t].x.row(n), atTime[t+1].x.row(n),
            atTime[t].vel.row(n), atTime[t+1].vel.row(n),
            frac_s, x, vel, accel
          );
          out.atTime[s].x.row(n) = x.transpose();
          out.atTime[s].vel.row(n) = vel.transpose();
          out.atTime[s].derived_accel.row(n) = accel.transpose();
        }

        if (assumeOneMaskEntry) return;
      }

      if (calc_groundForce) {
        out.atTime[s].groundForce_f = (1.-a)*atTime[t].groundForce_f + a*atTime[t+1].groundForce_f;
        if (assumeOneMaskEntry) return;
      }
      out.atTime[s].groundForce_c = atTime[t].groundForce_c;

      if (calc_manipForce) {
        out.atTime[s].manipForce = (1.-a)*atTime[t].manipForce + a*atTime[t+1].manipForce;
        if (assumeOneMaskEntry) return;
      }
      out.atTime[s].manipForce_c = atTime[t].manipForce_c;


                // sanity check at knots
      // if (s == tA) {
      //   cout << "======================\n";
      //   cout << "sanity: manipDofs" << out.atTime[s].manipDofs.transpose() << " | " << atTime[t].manipDofs.transpose() << endl;
      //   cout << "sanity: x" << out.atTime[s].x << " | " << atTime[t].x << endl;
      //   cout << "sanity: v" << out.atTime[s].vel << " | " << atTime[t].vel << endl;
      // }
      //  assert(out.atTime[s].manipDofs == atTime[t].manipDofs);

        //assert(out.atTime[s].x == atTime[t].x);
        //assert(out.atTime[s].vel == atTime[t].vel);
     // }
    }
  }
}

bool OptRopeState::StateAtTime::isApprox(const OptRopeState::StateAtTime &other) const {
  return dim() == other.dim()
      && manipDofs.isApprox(other.manipDofs)
      && x.isApprox(other.x)
      && vel.isApprox(other.vel)
      && groundForce_f.isApprox(other.groundForce_f)
      && groundForce_c.isApprox(other.groundForce_c)
      && manipForce.isApprox(other.manipForce)
      && manipForce_c.isApprox(other.manipForce_c);
}

bool OptRopeState::isApprox(const OptRopeState &other) const {
  if (atTime.size() != other.atTime.size()) {
    return false;
  }
  for (int t = 0; t < atTime.size(); ++t) {
    if (!atTime[t].isApprox(other.atTime[t])) {
      return false;
    }
  }
}
