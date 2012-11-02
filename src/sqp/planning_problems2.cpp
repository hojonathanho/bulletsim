#include "planning_problems.h"
#include "config_sqp.h"
#include "traj_sqp.h"
#include "simulation/openravesupport.h"

using namespace Eigen;
using namespace std;

static double COLL_COST_MULT = 3;
static double INIT_MAX_DIFF_PER_ITER = .25;

void setupArmToJointTarget(TrajOptimizer& opt, const VectorXd& endJoints, RobotManipulatorPtr manip) {

  VectorXd lower, upper;
  vector<int> armInds = manip->manip->GetArmIndices();
  getJointLimits(manip->robot->robot, armInds, lower, upper);
  VectorXd maxDiffPerIter = VectorXd::Constant(7, .25);
  VectorXd startJoints = toVectorXd(arm->getDOFValues());

  opt.setTrustRegion(TrustRegionPtr(new JointBounds(&opt, maxDiffPerIter, lower, upper)));
  opt.addCost(CostPtr(new CollisionCost(&opt, manip->robot, dofInds, false, SQPConfig::collCoefInit)));
  opt.addCost(CostPtr(new JntLenCost(&opt, SQPConfig::lengthCoef)));
  opt.initialize(linearInterp(startJoints, endJoints, SQPConfig::nStepsInit), arange(SQPConfig::nStepsInit));

}
void setupArmToCartTarget(TrajOptimizer& opt, const btTransform& goal, RobotManipulatorPtr arm) {
  vector<double> ikSoln;
  bool ikSuccess = arm->solveIKUnscaled(util::toRaveTransform(goalTrans), ikSoln);
  if (!ikSuccess) {
    LOG_ERROR("no ik solution for target!");
    return false;
  }
  VectorXd endJoints = toVectorXd(ikSoln);
  VectorXd startJoints = toVectorXd(arm->getDOFValues());

  VectorXd lower, upper;
  vector<int> armInds = manip->manip->GetArmIndices();
  getJointLimits(manip->robot->robot, armInds, lower, upper);
  VectorXd maxDiffPerIter = VectorXd::Constant(7, .25);

  static double POS_COEFF = 100;
  static double ROT_COEFF = 100;

  opt.setTrustRegion(TrustRegionPtr(new JointBounds(&opt, maxDiffPerIter, lower, upper)));
  opt.addCost(CostPtr(new CollisionCost(&opt, arm->robot, dofInds, false, SQPConfig::collCoefInit)));
  opt.addCost(CostPtr(new JntLenCost(&opt, SQPConfig::lengthCoef)));
  opt.addCost(CostPtr(new CartPoseCost(&opt, arm, NULL, dofInds, false, goalTrans, POS_COEFF, ROT_COEFF)));
  opt.initialize(linearInterp(startJoints, endJoints, SQPConfig::nStepsInit), arange(SQPConfig::nStepsInit));

}


void setupArmToGrasp(TrajOptimizer& opt, const btTransform& goal, RobotManipulatorPtr arm) {

}


void setupArmToCartTargetWithBase(TrajOptimizer& opt, const btTransform& goalTrans, RobotManipulatorPtr arm) {

}
