#include "optrope.h"

#include <nlopt.hpp>
#include <Eigen/Dense>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/timer.hpp>

#include "ophys_config.h"
#include "ophys_common.h"
#include "scenario_impl.h"

#include "simulation/simplescene.h"
#include "robots/pr2.h"

using namespace Eigen;
using namespace std;

using namespace ophys;


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
  // MatrixX3d initPositions(OPhysConfig::N, 3);
  // for (int i = 0; i < OPhysConfig::N; ++i) {
  //   initPositions.row(i) << (-1 + 2*i/(OPhysConfig::N-1.0)), 0, 0.05;
  // }
  // const int test_T = 3, test_N = 4;
  // double linklen = abs(initPositions(0, 0) - initPositions(1, 0));
  // Vector7d initManipPos(Vector7d::Zero());
  boost::shared_ptr<RopeLiftScenario> scenario(new RopeLiftScenario);
  //OptRope optrope(scenario);
  OptRope optrope(OPhysConfig::T, scenario);
  int test_T = OPhysConfig::T, test_N = scenario->getInitialRopePoints().rows();
  // optrope.setInitPositions(initPositions);
  // optrope.setInitManipDofs(initManipPos);
  OptRopeState testingState(&optrope, test_T, test_N);
  VectorXd testingCol = VectorXd::Random(testingState.dim());
  testingState.initFromColumn(testingCol);
  bool b;
  assert(b = testingCol == testingState.toColumn());
  if (b) {
    cout << "state/column conversion testing passed" << endl;
  }

  OptRopeState exp1 = testingState.expandByInterp(10);
  OptRopeState exp2 = testingState.expandByInterp(10);
  OptRopeState fullMask(&optrope, test_T, test_N); fullMask.initFromColumn(VectorXd::Ones(fullMask.dim()));
  testingState.fillExpansion(10, exp2, &fullMask);
  assert(b = exp1.isApprox(exp2));
  if (b) {
    cout << "expansion test passed" << endl;
  }

  for (int i = 0; i < fullMask.dim(); ++i) {
    VectorXd col(VectorXd::Zero(fullMask.dim()));
    col[i] = 1;
    fullMask.initFromColumn(col);
    testingState.initFromColumn(testingCol);
    OptRopeState exp4 = testingState.expandByInterp(10);
    testingState.initFromColumn(testingCol + col*rand());
    OptRopeState exp3 = testingState.expandByInterp(10);
    //testingState.fillExpansion(10, exp1);
    testingState.fillExpansion(10, exp4, &fullMask);
    // cout << "MASKMASKMASKMASKMASKMASK\n" << fullMask.toString()<< "\nMASKMASKMASKMASKMASKMASKMASKMASK" << endl;
    if (!exp3.isApprox(exp4)) {
      cout << i << endl;

      MatrixXd m(exp1.dim(), 3);
      m.col(0) = exp1.toColumn(); m.col(1) = exp1.toColumn();
      m.col(2) = m.col(0) - m.col(1);
      cout << m << "\n\n" << endl;
    }
    // cout << exp3.toString() << "\n==============\n" << exp4.toString() << endl;
    assert(b = b && exp3.isApprox(exp4));
  }
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


struct LocalConfig : public Config {
  static string out;
  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("out", &out, "output file"));
  }
};
string LocalConfig::out = "";


int main(int argc, char *argv[]) {
  Parser parser;
  parser.addGroup(OPhysConfig());
  parser.addGroup(LocalConfig());
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

    pr2m->pr2Left->setDOFValues(pr2LeftNeutralPos());
    pr2m->pr2Right->setDOFValues(pr2RightNeutralPos());
  }
  BulletObject::Ptr table = makeTable(initTableCornersWorld(), 0.01);
  scene.env->add(table);
  table->setColor(0, 1, 0, 0.2);

  // set up scenario for optimization
  // boost::shared_ptr<RopeLiftScenario> scenario(new RopeLiftScenario);
  // scenario->setDestPos0(centroid(scenario->getInitialRopePoints()) + Vector3d(0.2, 0, 0.1));
  //scenario->setDestPos0(scenario->getInitialRopePoints().row(0).transpose() + Vector3d(0.2, -.3, 0.2));
  //boost::shared_ptr<Scenario> scenario(new RopeDragScenario);
  boost::shared_ptr<Scenario> scenario(new PointManipScenario2);

  OptRope optrope(OPhysConfig::T, scenario);
  if (OPhysConfig::useRobot) {
    optrope.setRobot(pr2m->pr2, pr2m->pr2Right);
  }

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
cout << "outfile: " << LocalConfig::out << endl;
  if (LocalConfig::out != "") {
    finalState.writeToFile(LocalConfig::out);
    cout << "Wrote results to " << LocalConfig::out << endl;
  }

  return 0;
}
