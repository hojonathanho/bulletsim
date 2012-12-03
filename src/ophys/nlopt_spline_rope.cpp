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
#include "robots/pr2.h"

using namespace Eigen;
using namespace std;

using namespace ophys;


struct OptRopePlot {
  Scene *m_scene;
  PlotSpheres::Ptr m_plotSpheres;
  PlotLines::Ptr m_plotLines;
  const int m_N;
  bool m_useRobot;
  RobotManipulator::Ptr m_manip;

  OptRopePlot(int N, Scene *scene, const MatrixX3d &initPos, const Vector7d &initManipDofs)
    : m_N(N),
      m_scene(scene),
      m_plotSpheres(new PlotSpheres),
      m_plotLines(new PlotLines(5)),
      m_useRobot(false)
  {
    m_scene->env->add(m_plotSpheres);
    m_scene->env->add(m_plotLines);
    draw(initPos, initManipDofs);
  }

  void setRobot(RobotManipulator::Ptr manip) {
    m_useRobot = true;
    m_manip = manip;
  }

  void draw(const MatrixX3d &pos, const Vector7d &manipDofs) {
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
    if (!m_useRobot) {
      centers->push_back(osg::Vec3(manipDofs(0), manipDofs(1), manipDofs(2)));
      rgba->push_back(osg::Vec4(0, 1, 0, 0.7));
      radii.push_back(0.05);
    }

    m_plotSpheres->plot(centers, rgba, radii);

    // imaginary lines connecting rope control points
    osg::ref_ptr<osg::Vec3Array> linePts(new osg::Vec3Array());
    for (int i = 0; i < pos.rows() - 1; ++i) {
      linePts->push_back(osg::Vec3(pos(i, 0), pos(i, 1), pos(i, 2)));
      linePts->push_back(osg::Vec3(pos(i+1, 0), pos(i+1, 1), pos(i+1, 2)));
    }
    m_plotLines->setPoints(linePts);

    if (m_useRobot) {
      m_manip->setDOFValues(toStlVec(manipDofs));
    }
  }

  void playTraj(const OptRopeState &s, bool idlePerStep=false, bool printProgress=false) {
    for (int t = 0; t < s.atTime.size(); ++t) {
      if (printProgress) {
        cout << "showing step " << (t+1) << "/" << s.atTime.size() << endl;
      }
      draw(s.atTime[t].x, s.atTime[t].manipDofs);
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

static bool runTests() {
  MatrixX3d initPositions(OPhysConfig::N, 3);
  for (int i = 0; i < OPhysConfig::N; ++i) {
    initPositions.row(i) << (-1 + 2*i/(OPhysConfig::N-1.0)), 0, 0.05;
  }
  double linklen = abs(initPositions(0, 0) - initPositions(1, 0));
  Vector7d initManipPos(Vector7d::Zero());
  OptRope optrope(initPositions, initManipPos, OPhysConfig::T, OPhysConfig::N, linklen);
  OptRopeState testingState(optrope, 100, 200);
  VectorXd testingCol = VectorXd::Random(testingState.dim());
  testingState.initFromColumn(testingCol);
  bool b;
  assert(b = testingCol == testingState.toColumn());
  if (b) {
    cout << "state/column conversion testing passed" << endl;
  }

  OptRopeState exp1 = testingState.expandByInterp(10);
  OptRopeState exp2 = testingState.expandByInterp(10);
  OptRopeState fullMask(optrope, 100, 200); fullMask.initFromColumn(VectorXd::Ones(fullMask.dim()));
  testingState.fillExpansion(10, exp2, &fullMask);
  assert(b = exp1.isApprox(exp2));
  if (b) {
    cout << "expansion test passed" << endl;
  }

  fullMask.initFromColumn(VectorXd::Zero(fullMask.dim()));
  testingState.atTime[0].manipDofs[0] = 0.5;
  fullMask.atTime[0].manipDofs[0] = 1;
  testingState.fillExpansion(10, exp1);
  testingState.fillExpansion(10, exp2, &fullMask);
  assert(b = exp1.isApprox(exp2));
  if (b) {
    cout << "expansion+mask test passed" << endl;
  }

  
#if 0
  MatrixX3d initPositions(OPhysConfig::N, 3);
  for (int i = 0; i < OPhysConfig::N; ++i) {
    initPositions.row(i) << (-1 + 2*i/(OPhysConfig::N-1.0)), 0, 0.05;
  }
  double linklen = abs(initPositions(0, 0) - initPositions(1, 0));
  Vector3d initManipPos(0, 0, 2);
  OptRope optrope(initPositions, initManipPos, OPhysConfig::T, OPhysConfig::N, linklen);
  VectorXd initState = optrope.createInitState().toColumn();
  VectorXd grad_orig = optrope.costGrad_orig<VectorXd>(initState);
  VectorXd grad_new = optrope.costGrad<VectorXd>(initState);
  assert(b = grad_orig.isApprox(grad_new));
  if (b) {
    cout << "gradient test passed" << endl;
  }
#endif

  return true;
}

static vector<btVector3> initTableCornersWorld() {
  vector<btVector3> v;
  v.push_back(btVector3(OPhysConfig::tableDistFromRobot, -OPhysConfig::tableWidth/2, OPhysConfig::tableHeight));
  v.push_back(btVector3(OPhysConfig::tableDistFromRobot, OPhysConfig::tableWidth/2, OPhysConfig::tableHeight));
  v.push_back(btVector3(OPhysConfig::tableDistFromRobot + OPhysConfig::tableLength, OPhysConfig::tableWidth/2, OPhysConfig::tableHeight));
  v.push_back(btVector3(OPhysConfig::tableDistFromRobot + OPhysConfig::tableLength, -OPhysConfig::tableWidth/2, OPhysConfig::tableHeight));
  return v;
}

static BulletObject::Ptr makeTable(const vector<btVector3>& corners, float thickness) {
  btVector3 origin = (corners[0] + corners[2])/2;
  origin[2] -= thickness/2;
  btVector3 halfExtents = (corners[2] - corners[0]).absolute()/2;
  halfExtents[2] = thickness/2;

  return BulletObject::Ptr(new BoxObject(0,halfExtents,btTransform(btQuaternion(0,0,0,1),origin)));
}


int main(int argc, char *argv[]) {
  Parser parser;
  parser.addGroup(OPhysConfig());
  parser.read(argc, argv);

  if (OPhysConfig::runTests && !runTests()) {
    return 1;
  }

  // set up scene
  Scene scene;
  boost::shared_ptr<PR2Manager> pr2m;
  if (OPhysConfig::useRobot) {
    pr2m.reset(new PR2Manager(scene));
    vector<int> indices(1, pr2m->pr2->robot->GetJointIndex("torso_lift_joint"));
    vector<double> values(1, .31);
    pr2m->pr2->setDOFValues(indices, values);
  }
  BulletObject::Ptr table = makeTable(initTableCornersWorld(), 0.01);
  scene.env->add(table);
  table->setColor(0, 1, 0, 0.2);

  // set up optimization
  MatrixX3d initPositions(OPhysConfig::N, 3);
  Vector3d center(0, 0, 0);
  for (int i = 0; i < OPhysConfig::N; ++i) {
    initPositions.row(i) <<
      OPhysConfig::tableDistFromRobot+0.1,
      (-0.5 + 1.*i/(OPhysConfig::N-1.0)),
      OPhysConfig::tableHeight+0.05;
    center += initPositions.row(i);
  }
  center /= OPhysConfig::N;
  double linklen = (initPositions.row(0) - initPositions.row(1)).norm();
  //Vector3d initManipPos = center + Vector3d(0, 0, 2);
  Vector7d initManipPos(Vector7d::Zero());
  if (!OPhysConfig::useRobot) {
    initManipPos.head<3>() = center + Vector3d(0, 0, 2);
  }

  OptRope optrope(initPositions, initManipPos, OPhysConfig::T, OPhysConfig::N, linklen);
  if (OPhysConfig::useRobot) {
    optrope.setRobot(pr2m->pr2, pr2m->pr2Right);
  }


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

  OptRopePlot plot(OPhysConfig::N, &scene, finalState.atTime[0].x, finalState.atTime[0].manipDofs);
  if (OPhysConfig::useRobot) {
    plot.setRobot(pr2m->pr2Right);
  }

  scene.startViewer();
  plot.playTraj(finalState, true, true);

  scene.idle(true);

  return 0;
}
