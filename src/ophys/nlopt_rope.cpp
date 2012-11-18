#include <nlopt.hpp>
#include <Eigen/Dense>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/random.hpp>
#include <boost/timer.hpp>

#include "ophys_config.h"
#include "ophys_common.h"

#include "simulation/simplescene.h"
#include "simulation/plotting.h"

using namespace Eigen;
using namespace std;

using namespace ophys;


int COPY_A=0, COPY_B=0;
inline vector<double> toStlVec(const VectorXd &v) {
  COPY_A++;
  vector<double> out(v.size());
  for (int i = 0; i < v.size(); ++i) {
    out[i] = v[i];
  }
  return out;
}

inline VectorXd toEigVec(const vector<double> &v) {
  COPY_B++;
  VectorXd out(v.size());
  for (int i = 0; i < v.size(); ++i) {
    out[i] = v[i];
  }
  return out;
}

/*
void addNoise(VectorXd &x, double mean, double stddev) {
  static boost::mt19937 rng(static_cast<unsigned> (std::time(0)));
  boost::normal_distribution<double> norm_dist(mean, stddev);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > gen(rng, norm_dist);
  for (int i = 0; i < x.size(); ++i) {
    x(i) += gen();
  }
}*/

template<typename T>
void addNoise(T &x, double mean, double stddev) {
  static boost::mt19937 rng(static_cast<unsigned> (std::time(0)));
  boost::normal_distribution<double> norm_dist(mean, stddev);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > gen(rng, norm_dist);
  for (int i = 0; i < x.size(); ++i) {
    x[i] += gen();
  }
}


struct CostCoeffs {
  double groundPenetration;
  double velUpdate;
  double initialVelocity;
  double contact;
  double forces;
};

struct OptRope {
  const int m_N; // num particles
  const int m_T; // timesteps
  static const int m_dim = 5; // state dim for 1 particle for 1 timestep
  const MatrixX3d m_initPos; // initial positions of the points (row(n) is the position of point n)
  CostCoeffs m_coeffs;
  const double GROUND_HEIGHT;

  OptRope(const MatrixX3d &initPos, int N, int T) : m_N(N), m_T(T), m_initPos(initPos), GROUND_HEIGHT(0.) {
    assert(m_N >= 1);
    assert(m_T >= 1);
    assert(initPos.rows() == m_N);

    setCoeffs1();    
  }

  int getNumVariables() { return m_N * m_T * m_dim; }

  void setCoeffs1() {
    m_coeffs.groundPenetration = 1.;
    m_coeffs.velUpdate = 0.1;
    m_coeffs.initialVelocity = 1000.;
    m_coeffs.contact = 0.1;
    m_coeffs.forces = 1;
  }

  void setCoeffs2() {
    m_coeffs.groundPenetration = 1.;
    m_coeffs.velUpdate = 10.;
    m_coeffs.initialVelocity = 1000.;
    m_coeffs.contact = 1;
    m_coeffs.forces = 1;
  }

  ///// Accessors for state vectors /////
  inline int idx(int t, int n, int i) const {
    assert(0 <= t && t < m_T);
    assert(0 <= n && n < m_N);
    assert(0 <= i && i < m_dim);
    return i + n*m_dim + t*m_dim*m_N;
  }
  inline int idx_vel_x(int t, int n) const __attribute__((always_inline)) { return idx(t, n, 0); }
  inline int idx_vel_y(int t, int n) const { return idx(t, n, 1); }
  inline int idx_vel_z(int t, int n) const { return idx(t, n, 2); }

  inline int idx_groundForce_z(int t, int n) const { return idx(t, n, 3); }
  inline int idx_groundForce_c(int t, int n) const { return idx(t, n, 4); }

  inline Vector3d get_vel(const VectorXd &state, int t, int n) {
    return Vector3d(state[idx_vel_x(t,n)], state[idx_vel_y(t,n)], state[idx_vel_z(t,n)]);
  }

  void printState(const VectorXd &state, int t, int n) {
    cout << "t=" << t << " n=" << n << ": "
         << "v: " << get_vel(state,t,n).transpose() << " | "
         << state[idx_groundForce_z(t,n)] << ' '
         << state[idx_groundForce_c(t,n)] << '\n';
  }

  ///////////////////////////////////////


  typedef vector<MatrixX3d> PosVector;
  // return_value[t].row(n) is the position of point n at time t
  PosVector integrateVelocities(const VectorXd &state) {
   PosVector pos(m_T, MatrixX3d::Zero(m_N, 3));
    pos[0] = m_initPos;
    for (int t = 0; t < m_T - 1; ++t) {
      for (int n = 0; n < m_N; ++n) {
        pos[t+1].row(n) = pos[t].row(n) + get_vel(state, t+1, n).transpose()*OPhysConfig::dt;
      }
    }
    return pos;
  }

  VectorXd genInitState() {
    VectorXd x0 = VectorXd::Zero(getNumVariables());
    for (int t = 0; t < m_T; ++t) {
      for (int n = 0; n < m_N; ++n) {
        x0[idx_groundForce_c(t,n)] = 1;
      }
    }
    return x0;
  }


  ///////// Cost function components /////////////
  double cost_groundPenetration(const PosVector &pos) {
    // sum of squares of positive parts
    double cost = 0;
    for (int t = 0; t < m_T; ++t) {
      for (int n = 0; n < m_N; ++n) {
        cost += square(max(0., GROUND_HEIGHT - pos[t](n,2)));
      }
    }
    return cost;
  }

  double cost_velUpdate(const VectorXd &state, const PosVector &pos) {
    double cost = 0;
    for (int t = 0; t < m_T - 1; ++t) {
      for (int n = 0; n < m_N; ++n) {
        // compute total force on particle n at time t
        Vector3d groundForce(0, 0, state[idx_groundForce_z(t,n)]);
        Vector3d totalForce = OPhysConfig::gravity + groundForce;
        Vector3d vdiff = get_vel(state, t+1, n) - get_vel(state, t, n);
        cost += (vdiff - totalForce*OPhysConfig::dt).squaredNorm();
      }
    }
    return cost;
  }

  double cost_initialVelocity(const VectorXd &state) {
    double cost = 0;
    for (int n = 0; n < m_N; ++n) {
      cost += get_vel(state, 0, n).squaredNorm();
    }
    return cost;
  }

  double cost_contact(const VectorXd &state, const PosVector &pos) {
    double cost = 0;
    // ground force: groundContact^2 * (ground non-violation)^2
    for (int t = 0; t < m_T; ++t) {
      for (int n = 0; n < m_N; ++n) {
        //cost += square(state[idx_groundForce_c(t,n)]) * (square(pos[t](n,2)) + square(state[idx_vel_z(t,n)]));
        cost += square(state[idx_groundForce_c(t,n)]) * square(max(0., pos[t](n,2) - GROUND_HEIGHT));
      }
    }
    return cost;
  }

  double cost_forces(const VectorXd &state) {
    double cost = 0;
    // (force^2 / contact^2) + force^2
    for (int t = 0; t < m_T; ++t) {
      for (int n = 0; n < m_N; ++n) {
        cost += square(state[idx_groundForce_z(t,n)]) / (1e-5 + square(state[idx_groundForce_c(t,n)]));
        cost += 1e-3*square(state[idx_groundForce_z(t,n)]);
      }
    }
    return cost;
  }

  double costfunc(const VectorXd &state) {
    assert(state.size() == getNumVariables());

    PosVector pos = integrateVelocities(state);

    double cost = 0;

    cost += m_coeffs.groundPenetration * cost_groundPenetration(pos);
    cost += m_coeffs.velUpdate * cost_velUpdate(state, pos);
    cost += m_coeffs.initialVelocity * cost_initialVelocity(state);
    cost += m_coeffs.contact * cost_contact(state, pos);
    cost += m_coeffs.forces * cost_forces(state);

    return cost;
  }

  VectorXd costGrad(VectorXd &x0) {
    static const double eps = 1e-3;
    VectorXd grad(x0.size());
    for (int i = 0; i < x0.size(); ++i) {
      double xorig = x0(i);
      x0(i) = xorig + eps;
      double b = costfunc(x0);
      x0(i) = xorig - eps;
      double a = costfunc(x0);
      x0(i) = xorig;
      grad(i) = (b - a) / (2*eps);
    }
    return grad;
  }

  double nlopt_costWrapper(const vector<double> &x, vector<double> &grad) {
    VectorXd ex = toEigVec(x);
    if (!grad.empty()) {
      vector<double> g = toStlVec(costGrad(ex));
      assert(g.size() == grad.size());
      grad = g;
    }
    return costfunc(ex);
  }

  // vector<double> getLowerBounds() {
  //   static const double eps = 1e-6;
  //   vector<double> lb(getNumVariables(), -HUGE_VAL);
  //   for (int n = 0; n < m_N; ++n) {
  //     lb[idx_vel_x(0,n)] = -eps;
  //     lb[idx_vel_y(0,n)] = -eps;
  //     lb[idx_vel_z(0,n)] = -eps;
  //   }
  //   return lb;
  // }

  // vector<double> getUpperBounds() {
  //   static const double eps = 1e-6;
  //   vector<double> ub(getNumVariables(), HUGE_VAL);
  //   for (int n = 0; n < m_N; ++n) {
  //     ub[idx_vel_x(0,n)] = eps;
  //     ub[idx_vel_y(0,n)] = eps;
  //     ub[idx_vel_z(0,n)] = eps;
  //   }
  //   return ub;
  // }

};


struct OptRopePlot {
  Scene *m_scene;
  PlotSpheres::Ptr m_plotSpheres;
  const int m_N;

  OptRopePlot(int N, Scene *scene, const MatrixX3d &initPos) : m_N(N), m_scene(scene), m_plotSpheres(new PlotSpheres) {
    m_scene->env->add(m_plotSpheres);
    draw(initPos);
  }

  void draw(const MatrixX3d &pos) {
    osg::ref_ptr<osg::Vec3Array> centers(new osg::Vec3Array());
    osg::ref_ptr<osg::Vec4Array> rgba(new osg::Vec4Array());
    vector<float> radii;
    for (int i = 0; i < pos.rows(); ++i) {
      centers->push_back(osg::Vec3(pos(i, 0), pos(i, 1), pos(i, 2)));
      rgba->push_back(osg::Vec4(1, 0, 0, 1));
      radii.push_back(0.05);
    }
    m_plotSpheres->plot(centers, rgba, radii);
  }

  void playTraj(const OptRope::PosVector &pv, bool idlePerStep=false, bool printProgress=false) {
    for (int t = 0; t < pv.size(); ++t) {
      if (printProgress) {
        cout << "showing step " << (t+1) << "/" << pv.size() << endl;
      }
      draw(pv[t]);
      m_scene->step(0);
      if (idlePerStep) m_scene->idle(true);
    }
  }

};

static double nlopt_costWrapper(const vector<double> &x, vector<double> &grad, void *data) {
  OptRope *optrope = static_cast<OptRope *> (data);
  return optrope->nlopt_costWrapper(x, grad);
}

static void runOpt(nlopt::opt &opt, vector<double> &x0, double &minf) {
  try {
    opt.optimize(x0, minf);
  } catch (...) {

  }
}

int main() {
  const int N = 2;
  const int T = 20;

  MatrixX3d initPositions(N, 3);
  for (int i = 0; i < N; ++i) {
    initPositions.row(i) << (-1 + 2*i/(N-1.0)), 0, 0.05;
  }


  OptRope optrope(initPositions, N, T);
  cout << "optimizing " << optrope.getNumVariables() << " variables" << endl;
  nlopt::opt opt(nlopt::LD_LBFGS, optrope.getNumVariables());
  //opt.set_lower_bounds(optrope.getLowerBounds());
  //opt.set_upper_bounds(optrope.getUpperBounds());
  opt.set_min_objective(nlopt_costWrapper, &optrope);


  VectorXd initState = optrope.genInitState();

  boost::timer timer;
  vector<double> x0 = toStlVec(initState);
  double minf;

  optrope.setCoeffs1();
  addNoise(x0, 0, 0.01);
  nlopt::result result = opt.optimize(x0, minf);
  //cout << "solution: " << toEigVec(x0).transpose() << '\n';
  cout << "function value: " << minf << endl;
  cout << "time elapsed: " << timer.elapsed() << endl;

  optrope.setCoeffs2();
  addNoise(x0, 0, 0.01);
  runOpt(opt, x0, minf);
  cout << "function value: " << minf << endl;
  cout << "time elapsed: " << timer.elapsed() << endl;

  cout << COPY_A << ' ' << COPY_B << endl;

  for (int t = 0; t < optrope.m_T; ++t) {
    optrope.printState(toEigVec(x0), t, 0);
  }

  OptRope::PosVector soln = optrope.integrateVelocities(toEigVec(x0));
  Scene scene;
  OptRopePlot plot(N, &scene, soln[0]);
  scene.startViewer();
  plot.playTraj(soln, true, true);

  scene.idle(true);

  return 0;
}
