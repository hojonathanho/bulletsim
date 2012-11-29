#include "optrope.h"

#include <nlopt.hpp>
#include <Eigen/Dense>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/timer.hpp>

#include "ophys_config.h"
#include "ophys_common.h"

#include "simulation/simplescene.h"
#include "simulation/plotting.h"

using namespace Eigen;
using namespace std;

using namespace ophys;


struct OptRopePlot {
  Scene *m_scene;
  PlotSpheres::Ptr m_plotSpheres;
  PlotLines::Ptr m_plotLines;
  const int m_N;

  OptRopePlot(int N, Scene *scene, const MatrixX3d &initPos, const Vector3d &initManipPos)
    : m_N(N),
      m_scene(scene),
      m_plotSpheres(new PlotSpheres),
      m_plotLines(new PlotLines(5))
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

  void playTraj(const OptRopeState &s, bool idlePerStep=false, bool printProgress=false) {
    for (int t = 0; t < s.atTime.size(); ++t) {
      if (printProgress) {
        cout << "showing step " << (t+1) << "/" << s.atTime.size() << endl;
      }
      draw(s.atTime[t].x, s.atTime[t].manipPos);
      m_scene->step(0);
      if (idlePerStep) m_scene->idle(true);
    }
  }
};

static inline double nlopt_costWrapper(const vector<double> &x, vector<double> &grad, void *data) {
  return (static_cast<OptRope *> (data))->nlopt_costWrapper(x, grad);
}

static void runOpt(nlopt::opt &opt, vector<double> &x0, double &minf) {
  try {
    opt.optimize(x0, minf);
  } catch (...) {

  }
}

void runTests() {
  OptRopeState testingState(100, 200);
  VectorXd testingCol = VectorXd::Random(testingState.dim());
  testingState.initFromColumn(testingCol);
  assert(testingCol == testingState.toColumn());
  if (testingCol == testingState.toColumn()) {
    cout << "state/column conversion testing passed" << endl;
  }
}

int main(int argc, char *argv[]) {
  Parser parser;
  parser.addGroup(OPhysConfig());
  parser.read(argc, argv);

  if (OPhysConfig::runTests) {
    runTests();
  }

  MatrixX3d initPositions(OPhysConfig::N, 3);
  for (int i = 0; i < OPhysConfig::N; ++i) {
    initPositions.row(i) << (-1 + 2*i/(OPhysConfig::N-1.0)), 0, 0.05;
  }
  double linklen = abs(initPositions(0, 0) - initPositions(1, 0));
  Vector3d initManipPos(0, 0, 2);

  OptRope optrope(initPositions, initManipPos, OPhysConfig::T, OPhysConfig::N, linklen);



  // VectorXd x = optrope.genInitState().toColumn();

  // optrope.setCoeffs2();
  // addNoise(x, 0, 0.01);
  // OptRope::State finalState = optrope.solve(x);


  cout << "optimizing " << optrope.getNumVariables() << " variables" << endl;
  nlopt::opt opt(nlopt::LD_LBFGS, optrope.getNumVariables());
  opt.set_lower_bounds(toStlVec(optrope.getLowerBoundVec()));
  opt.set_upper_bounds(toStlVec(optrope.getUpperBoundVec()));
  opt.set_vector_storage(100000);
  opt.set_min_objective(nlopt_costWrapper, &optrope);


  VectorXd initState = optrope.createInitState().toColumn();

  boost::timer timer;
  vector<double> x0 = toStlVec(initState);
  double minf;

  optrope.setCoeffs1();
  optrope.addNoiseClamped(x0, 0, 0.01);
  //nlopt::result result = opt.optimize(x0, minf);
  runOpt(opt, x0, minf);
  //cout << "solution: " << toEigVec(x0).transpose() << '\n';
  cout << "function value: " << minf << endl;
  cout << "time elapsed: " << timer.elapsed() << endl;

  optrope.setCoeffs2();
  optrope.addNoiseClamped(x0, 0, 0.01);
  runOpt(opt, x0, minf);
  cout << "function value: " << minf << endl;
  cout << "time elapsed: " << timer.elapsed() << endl;


  OptRopeState finalState = optrope.toState(toEigVec(x0)).expandByInterp(OPhysConfig::interpPerTimestep);
  cout << finalState.toString() << endl;

  Scene scene;
  OptRopePlot plot(OPhysConfig::N, &scene, finalState.atTime[0].x, finalState.atTime[0].manipPos);
  scene.startViewer();
  plot.playTraj(finalState, true, true);

  scene.idle(true);

  return 0;
}
