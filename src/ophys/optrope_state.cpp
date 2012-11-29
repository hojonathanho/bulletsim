#include "optrope_state.h"

#include "splines.h"

#include <boost/format.hpp>
#include <string>
#include <sstream>
using namespace std;

OptRopeState::StateAtTime::StateAtTime(int N)
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

string OptRopeState::StateAtTime::toString() const {
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

OptRopeState OptRopeState::expandByInterp(int interpPerTimestep) const {
  const int origT = atTime.size(); const int N = atTime[0].m_N;
  const int newT = (origT - 1)*interpPerTimestep + 1;
  const ArrayXd tv(ArrayXd::LinSpaced(origT, 0, origT-1));
  const ArrayXd fine_tv(ArrayXd::LinSpaced(newT, 0, origT-1));

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
