#include "planning_problems.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "utils/vector_io.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "sqp_algorithm.h"
#include "config_sqp.h"
#include "sqp/config_sqp.h"
#include "sqp/kinematics_utils.h"


using namespace std;
using namespace Eigen;


static double COLL_COST_MULT = 3;


/**
 * @brief Solves a planning problem by adjusting parameters to make sure the
 * resulting trajectory is safe.
 *
 * Pseudocode:
 * 	while problem is not safe and within parameter limits
 * 	  problem.optimize()
 * 	  if problem is not discreteSafe
 * 	  	collisionCoefficient <- 2*collisionCoefficient
 * 	  else if problem is not continuousSafe
 * 	  	subdivide trajectory in segments that are in continuous collision
 * 	  if parameter limits violated
 * 	  	return false
 * 	return true
 *
 * @param prob the planning problem
 * @param cc collision cost calculator
 * @param allowedCollisionIntervals	time intervals in which collisions are allowed
 * @param maxCollCoeff the maximum collision coefficient to use
 * @param maxSteps the maximum number of steps in the trajectory
 */
bool outerOptimization(PlanningProblem& prob, CollisionCostPtr cc,
		const map<int, vector<paird> >& allowedCollisions) {
  for (int outerOptIter = 0;; ++outerOptIter) {
    LOG_INFO_FMT("outer optimization iteration: %i", outerOptIter);
    if (prob.m_tra->m_shrinkage < SQPConfig::shrinkLimit) prob.m_tra->adjustTrustRegion(10*SQPConfig::shrinkLimit / prob.m_tra->m_shrinkage);
    prob.optimize(SQPConfig::maxIter);

    // discrete is safe, else double coll coeff (but if it's at the upper limit, quit)
    // continuous is safe, else resample (but if you're at the max number of samples, quit)

    TrajCartCollInfo& discCollInfo = cc->m_cartCollInfo;

    if (!isSafe(discCollInfo, SQPConfig::distDiscSafe, prob.m_times,allowedCollisions)) { // not discrete safe
      if (cc->m_coeff < SQPConfig::maxCollCoef) {
      cc->m_coeff = fmin(cc->m_coeff * COLL_COST_MULT, SQPConfig::maxCollCoef);
        LOG_INFO_FMT("trajectory was not discrete-safe. collision coeff <- %.2f", cc->m_coeff);
        continue;
      }
      else {
        LOG_INFO("trajectory was not discrete-safe, but collision coeff is too high! stopping optimization");
        return false;
      }
    }
    else { // check continuous safety
      TrajCartCollInfo contCollInfo = continuousTrajCollisions(prob.m_currentTraj, cc->m_robot->robot, *cc->m_brs,
          cc->m_world, cc->m_dofInds, SQPConfig::distContSafe);
      vector<double> insertTimes = getSubdivisionTimes(contCollInfo, prob.m_times, allowedCollisions);
      //insertTimes = filterOutIntervals(insertTimes, allowedCollisionIntervals);
      if (insertTimes.size() > 0) {	// Not continuous safe
        LOG_INFO("trajectory was discrete-safe but not continuous-safe");
        if (prob.m_times.size() < SQPConfig::maxSteps) {
          assert(insertTimes.size() > 0);
          LOG_INFO("subdividing at times" << insertTimes);
          prob.subdivide(insertTimes);
          continue;
        }
        else {
          LOG_INFO("trajectory is too long to subdivide! stopping optimization");
          return false;
        }
      }
      else {
        LOG_INFO("hooray! trajectory is discrete-safe and continuous safe. stopping optimization");
        return true;
      }

    }
    ++outerOptIter;
  }
}

bool planArmToCartTarget(PlanningProblem& prob, const Eigen::VectorXd& startJoints, const btTransform& goalTrans, RaveRobotObject::Manipulator::Ptr arm, bool doOptimize) {
 /**
  * Generates a plan for moving the arm to a cartesian goal
  */
  BulletRaveSyncherPtr brs = syncherFromArm(arm);
  vector<double> ikSoln;
  bool ikSuccess = arm->solveIKUnscaled(util::toRaveTransform(goalTrans), ikSoln);
  if (!ikSuccess) {
    LOG_ERROR("no ik solution for target!");
    return false;
  }
  VectorXd endJoints = toVectorXd(ikSoln);

  MatrixXd initTraj = makeTraj(startJoints, endJoints, SQPConfig::nStepsInit);
  LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, false, defaultMaxStepMvmt(initTraj), SQPConfig::lengthCoef));
  CollisionCostPtr cc(new CollisionCost(arm->robot,   arm->robot->getEnvironment()->bullet->dynamicsWorld, brs,
      arm->manip->GetArmIndices(), SQPConfig::distPen, SQPConfig::collCoefInit));
  JointBoundsPtr jb(new JointBounds(true, false, defaultMaxStepMvmt(initTraj) / 5, arm->manip));
  CartesianPoseCostPtr cp(new CartesianPoseCost(arm, goalTrans, initTraj.rows() - 1, 1000., 1000));

  prob.initialize(initTraj, false);
  prob.addComponent(lcc);
  prob.addComponent(cc);
  prob.addTrustRegionAdjuster(jb);
  prob.addComponent(cp);
	if (doOptimize)
		return outerOptimization(prob, cc, map<int, vector<paird> >());
	else
		return true;
}

bool planArmToJointTarget(PlanningProblem& prob, const Eigen::VectorXd& startJoints, const Eigen::VectorXd& endJoints, RaveRobotObject::Manipulator::Ptr arm) {
  BulletRaveSyncherPtr brs = syncherFromArm(arm);
  MatrixXd initTraj = makeTraj(startJoints, endJoints, SQPConfig::nStepsInit); // xxx nsteps
  LOG_DEBUG("initial traj: \n" << initTraj);
  LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, true, defaultMaxStepMvmt(initTraj), SQPConfig::lengthCoef));
  CollisionCostPtr cc(new CollisionCost(arm->robot,   arm->robot->getEnvironment()->bullet->dynamicsWorld, brs,
      arm->manip->GetArmIndices(), SQPConfig::distPen, SQPConfig::collCoefInit));
  JointBoundsPtr jb(new JointBounds(true, true, defaultMaxStepMvmt(initTraj) / 5, arm->manip));
  prob.initialize(initTraj, true);
  prob.addComponent(lcc);
  prob.addComponent(cc);
  prob.addTrustRegionAdjuster(jb);
  return outerOptimization(prob, cc, map<int, vector<paird> >());
}

/**
 * @brief Adds steps to the end of the trajectory, copying the current last row
 * @param traj the trajectory to extend
 * @param nEnd the number of steps to add
 */
void addFixedEnd(MatrixXd& traj, int nEnd) {
  int oldLen = traj.rows();
  traj.conservativeResize(oldLen + nEnd, NoChange);
  for (int i=0; i < nEnd; ++i) traj.row(oldLen+i) = traj.row(oldLen-1);
}

VectorXd concatenate(VectorXd x0, VectorXd x1) {
  VectorXd out(x0.size()+x1.size());
  out.middleRows(0,x0.size()) = x0;
  out.middleRows(x0.size(),x1.size()) = x1;
  return out;
}



bool planArmToGrasp(PlanningProblem& prob, const Eigen::VectorXd& startJoints, const btTransform& goalTrans, RaveRobotObject::Manipulator::Ptr arm) {
  BulletRaveSyncherPtr brs = syncherFromArm(arm);
  int nEnd = 6;
  vector<double> ikSoln;
  bool ikSuccess = arm->solveIKUnscaled(util::toRaveTransform(goalTrans), ikSoln);
  if (!ikSuccess) {
    LOG_ERROR("no ik solution for target!");
    return false;
  }
  VectorXd endJoints = toVectorXd(ikSoln);

  MatrixXd initTraj = makeTraj(startJoints, endJoints, SQPConfig::nStepsInit);
  int oldLen = initTraj.rows();
  addFixedEnd(initTraj, nEnd);
  typedef pair<double, double> paird;
  vector<paird> allowedCollisionIntervals;
  allowedCollisionIntervals.push_back(paird(oldLen, oldLen+nEnd));
  int gripperInd = arm->manip->GetEndEffector()->GetIndex();

  map<int, vector<paird> > allowedCollisions;
  allowedCollisions[gripperInd] = allowedCollisionIntervals;

  LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, false, defaultMaxStepMvmt(initTraj), SQPConfig::lengthCoef));
  CollisionCostPtr cc(new CollisionCost(arm->robot,   arm->robot->getEnvironment()->bullet->dynamicsWorld, brs,
      arm->manip->GetArmIndices(), SQPConfig::distPen, SQPConfig::collCoefInit));
  JointBoundsPtr jb(new JointBounds(true, false, defaultMaxStepMvmt(initTraj) / 5, arm->manip));
  CartesianPoseCostPtr cp(new CartesianPoseCost(arm, goalTrans, initTraj.rows() - 1, 10000., 10000));
  CartesianVelConstraintPtr cvc(new CartesianVelConstraint(arm, oldLen, oldLen + nEnd, .015));

  prob.initialize(initTraj, false);
  prob.addComponent(lcc);
  prob.addComponent(cc);
  prob.addTrustRegionAdjuster(jb);
  prob.addComponent(cp);
  prob.addComponent(cvc);
  return outerOptimization(prob, cc, allowedCollisions);

}


bool planArmBaseToCartTarget(PlanningProblem& prob, const Eigen::VectorXd& startJoints, const btTransform& goalTrans,
                             RaveRobotObject::Manipulator::Ptr arm) {
  BulletRaveSyncherPtr brs = fullBodySyncher(arm->robot);

  VectorXd endJoints = startJoints;
  Vector2d goalXY(goalTrans.getOrigin().x(), goalTrans.getOrigin().y());
  Vector2d goalDir = (goalXY - startJoints.middleRows(7,2)).normalized();
  endJoints.middleRows(7,2) = goalXY - goalDir*1.; // 1 meter behind goal

  MatrixXd initTraj = makeTraj(startJoints, endJoints, SQPConfig::nStepsInit); // xxx nsteps
  VectorXd maxStepMvmt = defaultMaxStepMvmt(initTraj);
  maxStepMvmt.middleRows(7,2).setConstant(2);


  CartesianPoseCostPtr cp(new CartesianPoseCost(arm, goalTrans, initTraj.rows() - 1, 1000.,0));
  LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, false, maxStepMvmt, SQPConfig::lengthCoef));
  CollisionCostPtr cc(new CollisionCost(arm->robot,   arm->robot->getEnvironment()->bullet->dynamicsWorld, brs,
      arm->manip->GetArmIndices(), SQPConfig::distPen, SQPConfig::collCoefInit,true));
  JointBoundsPtr jb(new JointBounds(true, false, maxStepMvmt, arm->manip,3));
  jb->m_jointUpperLimit(9) = 1e-2; // since rotation is screwy
  jb->m_jointLowerLimit(9) = -1e-2;
  cp->m_l1 = true;

  prob.initialize(initTraj, false);
  prob.addComponent(lcc);
  prob.addComponent(cc);
  prob.addTrustRegionAdjuster(jb);
  prob.addComponent(cp);
//  prob.testObjectives();
  return outerOptimization(prob, cc, map<int, vector<paird> >());
}


#if 0
bool planArmToGrasp(PlanningProblem& prob, btTransform& target) {

  vector<double> ikSoln;
  bool ikSuccess = arm->solveIKUnscaled(toRaveTransform(goalTrans), ikSoln);
  if (!ikSucces) {
    LOG_ERROR("no ik solution for target!");
    return false;
  }
  VectorXd endJoints = toVectorXd(ikSolns[0]);

  MatrixXd initTraj = makeTraj(startJoints, endJoints, nInitialSteps);
  addFixedEnd(initTraj, nEnd);
  typedef pair<double, double> paird;
  vector<paird> allowedCollisionIntervals;
  allowedCollisionIntervals.push_back(paird(nInitialSteps, nInitialSteps + nEnd - 1));

  // todo: properly subdivide collision coeff

  LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, true, defaultMaxStepMvmt(
      initTraj), SQPConfig::lengthCoef));
  CollisionCostPtr cc(new CollisionCost(pr2->robot, scene.env->bullet->dynamicsWorld, brs,
      rarm->manip->GetArmIndices(), -BulletConfig::linkPadding / 2, SQPConfig::collCoefInit));
  JointBoundsPtr jb(new JointBounds(true, true, defaultMaxStepMvmt(initTraj) / 5, rarm->manip));
  CartesianPoseCostPtr cp(new CartesianPoseCost(arm, goalTrans, initTraj.rows() - 1, 100., 100));
  CartesianVelConstraintPtr cvc(new CartesianVelConstraint(arm, oldLen, oldLen + nFinal, .02*METERS));

  prob.initialize(initTraj, true);
  prob.addComponent(lcc);
  prob.addComponent(cc);
  prob.addComponent(jb);
  prob.addComponent(cp);
  prob.addComponent(cvc);

  return outerOptimiation(prob, xxx);

}
#endif
