#include <nlopt.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/random.hpp>
#include <boost/timer.hpp>

#include "ophys_config.h"
#include "ophys_common.h"
#include "ipopt_interface.h"

#include "simulation/simplescene.h"
#include "simulation/plotting.h"

using namespace Eigen;
using namespace std;

using namespace ophys;


static int COPY_A = 0, COPY_B = 0;
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

template<typename EigenType>
struct EigVector {
  typedef std::vector<EigenType, Eigen::aligned_allocator<EigenType> > type;
};

template<typename VectorType>
static void addNoise(VectorType &x, double mean, double stddev) {
  static boost::mt19937 rng(static_cast<unsigned> (std::time(0)));
  boost::normal_distribution<double> norm_dist(mean, stddev);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > gen(rng, norm_dist);
  for (int i = 0; i < x.size(); ++i) {
    x[i] += gen();
  }
}



struct OptRope {
  const int m_N; // num particles
  const int m_T; // timesteps
  const double m_linkLen; // resting distance

  const MatrixX3d m_initPos; // initial positions of the points (row(n) is the position of point n)
  const Vector3d m_initManipPos;

  struct CostCoeffs {
    double groundPenetration;
    double velUpdate;
    double initialVelocity;
    double contact;
    double forces;
    double linkLen;
    double goalPos;
    double manipSpeed;
    double initManipPos;
  } m_coeffs;
  const double GROUND_HEIGHT;

  OptRope(const MatrixX3d &initPos, const Vector3d &initManipPos, int T, int N, double linkLen)
    : m_N(N), m_T(T), m_initPos(initPos), m_initManipPos(initManipPos), m_linkLen(linkLen), GROUND_HEIGHT(0.)
  {
    assert(m_N >= 1);
    assert(m_T >= 1);
    assert(initPos.rows() == m_N);
    setCoeffs1();    
  }

  void setCoeffs1() {
    m_coeffs.groundPenetration = 1.;
    m_coeffs.velUpdate = 0.1;
    m_coeffs.initialVelocity = 1000.;
    m_coeffs.contact = 0.1;
    m_coeffs.forces = 1;
    m_coeffs.linkLen = 0.1;
    m_coeffs.goalPos = 1;
    m_coeffs.manipSpeed = 0.1;
    m_coeffs.initManipPos = 1;
  }

  void setCoeffs2() {
    m_coeffs.groundPenetration = 1.;
    m_coeffs.velUpdate = 10.;
    m_coeffs.initialVelocity = 1000.;
    m_coeffs.contact = 1;
    m_coeffs.forces = 1;
    m_coeffs.linkLen = 1;
    m_coeffs.goalPos = 1;
    m_coeffs.manipSpeed = 0.1;
    m_coeffs.initManipPos = 1;
  }

  struct State {

    struct StateAtTime {
      Vector3d manipPos; // pos of manipulator 'gripper'
    
      MatrixX3d vel; // velocities of points (ith row is the velocity of the ith point)
      VectorXd groundForce_f; // magnitudes of ground forces for each point
      VectorXd groundForce_c; // contact vars for ground forces

      MatrixX3d manipForce; // manipulator forces for each point
      VectorXd manipForce_c; // contact vars for manip forces

      explicit StateAtTime(int N)
        : manipPos(Vector3d::Zero()),
          vel(MatrixX3d::Zero(N, 3)),
          groundForce_f(VectorXd::Zero(N)),
          groundForce_c(VectorXd::Zero(N)),
          manipForce(MatrixX3d::Zero(N, 3)),
          manipForce_c(VectorXd::Zero(N))
      {
        m_N = N;
        m_dim = manipPos.size()
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

      void setAsLowerBound(int t) {
        manipPos.setConstant(-BoxConstrainedOptProblem::INF);
        vel.setConstant(t == 0 ? 0 : -BoxConstrainedOptProblem::INF);
        groundForce_f.setZero();
        groundForce_c.setConstant(-BoxConstrainedOptProblem::INF);
        manipForce.setConstant(-BoxConstrainedOptProblem::INF);
        manipForce_c.setConstant(-BoxConstrainedOptProblem::INF);
      }

      void setAsUpperBound(int t) {
        manipPos.setConstant(BoxConstrainedOptProblem::INF);
        vel.setConstant(t == 0 ? 0 : BoxConstrainedOptProblem::INF);
        groundForce_f.setConstant(BoxConstrainedOptProblem::INF);
        groundForce_c.setConstant(BoxConstrainedOptProblem::INF);
        manipForce.setConstant(BoxConstrainedOptProblem::INF);
        manipForce_c.setConstant(BoxConstrainedOptProblem::INF);
      }

      VectorXd toSubColumn() const {
        VectorXd col(dim());
        int pos = 0;
        col.segment<3>(pos) = manipPos; pos += 3;
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
          boost::format("> manipPos: %s\n> vel:\n%s\n> groundForce_f:%f\n> groundForce_c:%f\n> manipForce:\n%s\n> manipForce_c:%f\n")
            % manipPos.transpose()
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

    explicit State(int T, int N) : atTime(T, StateAtTime(N)) { }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int dim() const { return atTime[0].dim() * atTime.size(); }

    // Conversion/utility methods

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

    void setAsLowerBound() {
      for (int t = 0; t < atTime.size(); ++t) {
        atTime[t].setAsLowerBound(t);
      }
    }

    void setAsUpperBound() {
      for (int t = 0; t < atTime.size(); ++t) {
        atTime[t].setAsUpperBound(t);
      }
    }

    //string toString(int t, int n) const {
    string toString() const {
      stringstream ss;
      for (int t = 0; t < atTime.size(); ++t) {
        ss << "t=" << t << ":\n";
        ss << atTime[t].toString() << '\n';
      }
      return ss.str();
/*      return (
        boost::format("t=%d n=%d: v=[%s] | ground_f,c=(%f,%f) | manip_f,c=(%s,%f)")
        % t
        % n
        % atTime[t].vel.row(n)
        % atTime[t].groundForce_f[n]
        % atTime[t].groundForce_c[n]
        % atTime[t].manipForce.row(n)
        % atTime[t].manipForce_c[n]
      ).str();*/
    }
  };


  template<typename Derived>
  State toState(const DenseBase<Derived> &col) const {
    State s(m_T, m_N);
    s.initFromColumn(col);
    return s;
  }


  State genLowerBound() const {
    static State lb(m_T, m_N);
    lb.setAsLowerBound();
    return lb;
  }

  State genUpperBound() const {
    static State ub(m_T, m_N);
    ub.setAsUpperBound();
    return ub;
  }

  State genInitState() const {
    static State s(m_T, m_N);
    s.atTime[0].manipPos = m_initManipPos.transpose();
    return s;
  }

  int getNumVariables() const {
    static const State s(m_T, m_N);
    return s.dim();
  }


  typedef EigVector<MatrixX3d>::type PosVector;
  // return_value[t].row(n) is the position of point n at time t
  PosVector integrateVelocities(const State &state) const {
   PosVector pos(m_T, MatrixX3d::Zero(m_N, 3));
    pos[0] = m_initPos;
    for (int t = 0; t < m_T - 1; ++t) {
      for (int n = 0; n < m_N; ++n) {
        pos[t+1].row(n) = pos[t].row(n) + state.atTime[t+1].vel.row(n)*OPhysConfig::dt;
      }
    }
    return pos;
  }

  ///////// Cost function components /////////////
  double cost_groundPenetration(const PosVector &pos) const {
    // sum of squares of positive parts
    double cost = 0;
    for (int t = 0; t < m_T; ++t) {
      for (int n = 0; n < m_N; ++n) {
        cost += square(max(0., GROUND_HEIGHT - pos[t](n,2)));
      }
    }
    return cost;
  }

  double cost_velUpdate(const State &state, const PosVector &pos) const {
    double cost = 0;
    for (int t = 0; t < m_T - 1; ++t) {
      for (int n = 0; n < m_N; ++n) {
        // compute total force on particle n at time t
        Vector3d groundForce(0, 0, state.atTime[t].groundForce_f[n]);
        Vector3d manipForce = state.atTime[t].manipForce.row(n).transpose(); //(state.atTime[t].manipPos - pos[t].row(n).transpose()).normalized() * state.atTime[t].manipForce_f[n];
        Vector3d totalForce = OPhysConfig::gravity + groundForce + manipForce;
        Vector3d vdiff = (state.atTime[t+1].vel.row(n) - state.atTime[t].vel.row(n)).transpose();
        cost += (vdiff - totalForce*OPhysConfig::dt).squaredNorm();
      }
    }
    return cost;
  }

  double cost_initialVelocity(const State &state) const {
    double cost = 0;
    for (int n = 0; n < m_N; ++n) {
      cost += state.atTime[0].vel.row(n).squaredNorm();
    }
    return cost;
  }

  double cost_initManipPos(const State &state) const {
    return (state.atTime[0].manipPos - m_initManipPos).squaredNorm();
  }

  double cost_contact(const State &state, const PosVector &pos) const {
    double cost = 0;
    // ground force: groundContact^2 * (ground non-violation)^2
    for (int t = 0; t < m_T; ++t) {
      for (int n = 0; n < m_N; ++n) {
        //cost += square(state[idx_groundForce_c(t,n)]) * (square(pos[t](n,2)) + square(state[idx_vel_z(t,n)]));

        // ground contact
        cost += square(state.atTime[t].groundForce_c[n]) * square(max(0., pos[t](n,2) - GROUND_HEIGHT));

        // manipulator contact
        cost += square(state.atTime[t].manipForce_c[n]) * (state.atTime[t].manipPos - pos[t].row(n).transpose()).squaredNorm();
      }
    }
    return cost;
  }

  double cost_forces(const State &state) const {
    double cost = 0;
    for (int t = 0; t < m_T; ++t) {
      for (int n = 0; n < m_N; ++n) {
        // ground force (force^2 / contact^2) + force^2
        cost += square(state.atTime[t].groundForce_f[n]) / (1e-5 + square(state.atTime[t].groundForce_c[n]));
        cost += 1e-3*square(state.atTime[t].groundForce_f[n]);

        // manipulator force
        double manip_fsqnorm = state.atTime[t].manipForce.row(n).squaredNorm();
        cost += manip_fsqnorm / (1e-5 + square(state.atTime[t].manipForce_c[n]));
        cost += 1e-6*manip_fsqnorm;
      }
    }
    return cost;
  }

  // rope segment distance violation cost
  double cost_linkLen(const PosVector &pos) const {
    double cost = 0;
    for (int t = 0; t < m_T; ++t) {
      for (int n = 0; n < m_N - 1; ++n) {
        double dist = (pos[t].row(n) - pos[t].row(n+1)).norm();
        cost += square(dist - m_linkLen);
      }
    }
    return cost;
  }

  // goal cost: point 0 should be at ()
  double cost_goalPos(const PosVector &pos) const {
    static const Vector3d desiredPos0 = Vector3d(0, 0, 1); //m_initPos.row(0).transpose() + Vector3d(1);
    return (pos[m_T-1].row(0) - desiredPos0.transpose()).squaredNorm();
  }

  double cost_manipSpeed(const State &state) const {
    double cost = 0;
    for (int t = 0; t < m_T - 1; ++t) {
      cost += (state.atTime[t+1].manipPos - state.atTime[t].manipPos).squaredNorm();
    }
    return cost;
  }

  double costfunc(const State &state) const {
    PosVector pos = integrateVelocities(state);

    double cost = 0;

    cost += m_coeffs.groundPenetration * cost_groundPenetration(pos);
    cost += m_coeffs.velUpdate * cost_velUpdate(state, pos);
    cost += m_coeffs.initialVelocity * cost_initialVelocity(state);
    cost += m_coeffs.contact * cost_contact(state, pos);
    cost += m_coeffs.forces * cost_forces(state);
    //cost += m_coeffs.linkLen * cost_linkLen(pos);
    //cost += m_coeffs.goalPos * cost_goalPos(pos);
    //cost += m_coeffs.manipSpeed * cost_manipSpeed(state);
    //cost += m_coeffs.initManipPos * cost_initManipPos(state);

    return cost;
  }

  double costfunc_wrapper(const Eigen::Map<const VectorXd> &x) {
    State state(m_T, m_N);
    state.initFromColumn(x);
    return costfunc(state);
  }

  State solve() {
    return solve(genInitState());
  }

  State solve(const State &init) {
    return solve(init.toColumn());
  }

  State solve(const VectorXd &init) {
    cout << "initial vals: " << init.transpose() << endl;
    cout << "lb: " << genLowerBound().toColumn().transpose() << endl;
    cout << "ub: " << genUpperBound().toColumn().transpose() << endl;
    SmartPtr<TNLP> prob = new BoxConstrainedOptProblem(
      getNumVariables(),
      genLowerBound().toColumn(),
      genUpperBound().toColumn(),
      init,
      boost::bind(&OptRope::costfunc_wrapper, this, _1)
    );

    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    app->Options()->SetNumericValue("tol", 1e-3);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    app->Options()->SetIntegerValue("print_level", 5);
    app->Options()->SetIntegerValue("max_iter", 10000);

    ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Solve_Succeeded) {
      std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
      throw std::runtime_error((boost::format("ipopt initialization error: %d") % status).str());
    }
    status = app->OptimizeTNLP(prob);
    if (status == Solve_Succeeded) {
      std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
    } else {
      std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
    }

    const BoxConstrainedOptProblem::Solution &soln = ((BoxConstrainedOptProblem*) GetRawPtr(prob))->solution();
    State s(m_T, m_N);
    s.initFromColumn(soln.x);
    return s;
  }

  template<typename VecType, typename Derived>
  VecType costGrad(DenseBase<Derived> &x0) const {
    static const double eps = 1e-3;
    VecType grad(x0.size());
    State tmpstate(m_T, m_N);
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
    State state(m_T, m_N); state.initFromColumn(ex);
    if (!grad.empty()) {
      vector<double> g = costGrad<vector<double> >(ex);
      assert(g.size() == grad.size());
      grad = g;
    }
    return costfunc(state);
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
  PlotLines::Ptr m_plotLines;
  const int m_N;

  OptRopePlot(int N, Scene *scene, const MatrixX3d &initPos, const Vector3d &initManipPos)
    : m_N(N),
      m_scene(scene),
      m_plotSpheres(new PlotSpheres),
      m_plotLines(new PlotLines(2))
  {
    m_scene->env->add(m_plotSpheres);
    m_scene->env->add(m_plotLines);
    draw(initPos, initManipPos);
  }

  void draw(const MatrixX3d &pos, const Vector3d &manipPos) {
    // rope control points
    osg::ref_ptr<osg::Vec3Array> centers(new osg::Vec3Array());
    osg::ref_ptr<osg::Vec4Array> rgba(new osg::Vec4Array());
    vector<float> radii;
    for (int i = 0; i < pos.rows(); ++i) {
      centers->push_back(osg::Vec3(pos(i, 0), pos(i, 1), pos(i, 2)));
      double a = (double)i/(double)pos.rows();
      rgba->push_back(osg::Vec4(a, 0, 1.-a, 1));
      radii.push_back(0.03);
    }

    // manipulator position
    centers->push_back(osg::Vec3(manipPos(0), manipPos(1), manipPos(2)));
    rgba->push_back(osg::Vec4(0, 1, 0, 0.7));
    radii.push_back(0.05);

    m_plotSpheres->plot(centers, rgba, radii);


    // imaginary lines connecting rope control points
    osg::ref_ptr<osg::Vec3Array> linePts(new osg::Vec3Array());
    for (int i = 0; i < pos.rows() - 1; ++i) {
      linePts->push_back(osg::Vec3(pos(i, 0), pos(i, 1), pos(i, 2)));
      linePts->push_back(osg::Vec3(pos(i+1, 0), pos(i+1, 1), pos(i+1, 2)));
    }
    m_plotLines->setPoints(linePts);
  }

  void playTraj(const OptRope::PosVector &pv, const MatrixX3d &manipPositions, bool idlePerStep=false, bool printProgress=false) {
    assert(pv.size() == manipPositions.rows());
    for (int t = 0; t < pv.size(); ++t) {
      if (printProgress) {
        cout << "showing step " << (t+1) << "/" << pv.size() << endl;
      }
      draw(pv[t], manipPositions.row(t).transpose());
      m_scene->step(0);
      if (idlePerStep) m_scene->idle(true);
    }
  }
};

static double nlopt_costWrapper(const vector<double> &x, vector<double> &grad, void *data) {
  static int calls = 0;
  ++calls;
  if (calls % 1000 == 0) cout << "nlopt cost calls: " << calls << endl;
  OptRope *optrope = static_cast<OptRope *> (data);
  double val = optrope->nlopt_costWrapper(x, grad);
  if (calls % 1000 == 0)  cout << "val: " << val << endl;
  return val;
}

static void runOpt(nlopt::opt &opt, vector<double> &x0, double &minf) {
  try {
    opt.optimize(x0, minf);
  } catch (...) {

  }
}

int main() {
  const int N = 2;
  const int T = 10;

  MatrixX3d initPositions(N, 3);
  for (int i = 0; i < N; ++i) {
    initPositions.row(i) << (-1 + 2*i/(N-1.0)), 0, 0.05;
  }
  double linklen = abs(initPositions(0, 0) - initPositions(1, 0));
  Vector3d initManipPos(0, 0, 2);

  OptRope optrope(initPositions, initManipPos, T, N, linklen);



  // VectorXd x = optrope.genInitState().toColumn();

  // optrope.setCoeffs2();
  // addNoise(x, 0, 0.01);
  // OptRope::State finalState = optrope.solve(x);









  cout << "optimizing " << optrope.getNumVariables() << " variables" << endl;
  nlopt::opt opt(nlopt::LD_LBFGS, optrope.getNumVariables());
  //opt.set_lower_bounds(optrope.getLowerBounds());
  //opt.set_upper_bounds(optrope.getUpperBounds());
  opt.set_vector_storage(100000);
  opt.set_min_objective(nlopt_costWrapper, &optrope);


  VectorXd initState = optrope.genInitState().toColumn();

  boost::timer timer;
  vector<double> x0 = toStlVec(initState);
  double minf;

  optrope.setCoeffs1();
  addNoise(x0, 0, 0.01);
  //nlopt::result result = opt.optimize(x0, minf);
  runOpt(opt, x0, minf);
  //cout << "solution: " << toEigVec(x0).transpose() << '\n';
  cout << "function value: " << minf << endl;
  cout << "time elapsed: " << timer.elapsed() << endl;

  optrope.setCoeffs2();
  addNoise(x0, 0, 0.01);
  runOpt(opt, x0, minf);
  cout << "function value: " << minf << endl;
  cout << "time elapsed: " << timer.elapsed() << endl;

  cout << COPY_A << ' ' << COPY_B << endl;


  OptRope::State finalState = optrope.toState(toEigVec(x0));
  //for (int t = 0; t < optrope.m_T; ++t) {
    //cout << finalState.toString(t, 0) << endl;
    cout << finalState.toString() << endl;
  //}

  OptRope::PosVector soln = optrope.integrateVelocities(finalState);
  MatrixX3d manipPositions(T, 3);
  for (int t = 0; t < T; ++t) {
    manipPositions.row(t) = finalState.atTime[t].manipPos;
  }

  Scene scene;
  OptRopePlot plot(N, &scene, soln[0], finalState.atTime[0].manipPos);
  scene.startViewer();
  plot.playTraj(soln, manipPositions, true, true);

  scene.idle(true);

  return 0;
}
