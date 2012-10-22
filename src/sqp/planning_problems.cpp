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
 * Returns true if d is within one of the given intervals
 * @param d the floating point number to test
 * @param intervals a list of intervals to test
 * @return true if d is within one of the given intervals
 */
bool inSomeInterval(double d, const std::vector<paird>& intervals) {
  bool included = false;
  BOOST_FOREACH(const paird& interval, intervals) {
    if (d >= interval.first && d <= interval.second) included=true;
  }
  return included;
}
/**
 * Removes numbers from in which fall within the specified intervals
 * @param in input vector of doubles to be filtered
 * @param exclude vector of intervals to be excluded from the output
 * @return the filtered vector of doubles
 */
vector<double> filterOutIntervals(const std::vector<double>& in, const std::vector<paird>& exclude) {
  vector<double> out;
  BOOST_FOREACH(const double d, in) if (!inSomeInterval(d, exclude)) out.push_back(d);
  return out;
}

bool linkInAllowedCollision(const LinkCollision lc, double time,
    const std::vector<AllowedLinkCollision>& allowedCollisions){
  BOOST_FOREACH(const AllowedLinkCollision alc, allowedCollisions){
    if(alc.linkIndex == lc.linkInd){
      if(inSomeInterval(time, alc.intervals)){
        return true;
      }
    }
  }
  return false;
}
/**
 * Returns whether the given trajectory is safe according to the given minimum
 * distance from collisions, optionally ignoring collisions in specified
 * time intervals of the trajectory
 *
 * @param cci Collision information for the trajectory
 * @param distSafe minimum distance from potential collisions
 * @param times a vector of time values for the trajectory
 * @param allowedCollisions time intervals in which collisions are allowed
 */
//TODO: Perhaps there should be a trajectory class, and this should be a member
//TODO: Allow specific links to collide rather than time intervals
bool isSafe(const TrajCartCollInfo& cci, double distSafe, const Eigen::VectorXd& times,
		const std::vector<AllowedLinkCollision>& allowedCollisions) {
//  countCollisions(const TrajJointCollInfo& trajCollInfo, double safeDist, int& nNear, int& nUnsafe, int& nColl)
  assert(cci.size() == times.size());
  vector<double> discreteUnsafeTimes;
  for (int i=0; i < cci.size(); ++i) {
	  BOOST_FOREACH(const LinkCollision& lc, cci[i]) {
	    bool linkInAllowedCollisions = linkInAllowedCollision(lc, times(i), allowedCollisions);
		  if(!linkInAllowedCollisions && lc.dist < distSafe){
		    discreteUnsafeTimes.push_back(times(i));
		  }
	  }
  }
  if (discreteUnsafeTimes.size() >0) {
    LOG_INFO("steps with a discrete collision: " << discreteUnsafeTimes);
    stringstream ss;
//    BOOST_FOREACH(paird p, allowedCollisionIntervals) ss << "(" << p.first << "," << p.second << ") ";
//    LOG_INFO("allowed collision intervals:" << ss.str());
  }
  return discreteUnsafeTimes.size()==0;
}

/**
 * Given a trajectory with continuous collisions, this function decides when to
 * add points to the trajectory to form a safe trajectory
 * @param cci collision information for a trajectory
 * @param times time information for a trajectory
 * @return a list of new times to be sampled in a new trajectory
 */
vector<double> getSubdivisionTimes(const TrajCartCollInfo& cci, const Eigen::VectorXd& times,
    const std::vector<AllowedLinkCollision>& allowedCollisions) {
  int nContColl=0;
  vector<double> insertTimes;
  for (int i = 0; i < cci.size() - 1; ++i) {
    nContColl += cci[i].size();
    if (cci[i].size() > 0){
      BOOST_FOREACH(const LinkCollision& lc, cci[i]) {
        // Check if this link is allowed to collide
        // (the distance is meaningless for continuous collisions)
        bool linkInAllowedCollisions = linkInAllowedCollision(lc, times(i), allowedCollisions);
        if(!linkInAllowedCollisions){
          insertTimes.push_back((times(i) + times(i+1)) / 2);
        }
      }
    }
  }
  return insertTimes;
}

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
		const std::vector<AllowedLinkCollision>& allowedCollisions) {
  for (int outerOptIter = 0;; ++outerOptIter) {
    LOG_INFO_FMT("outer optimization iteration: %i", outerOptIter);
    if (prob.m_tra->m_shrinkage < SQPConfig::shrinkLimit) prob.m_tra->m_shrinkage = 10*SQPConfig::shrinkLimit;
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
      TrajCartCollInfo contCollInfo = continuousTrajCollisions(prob.m_currentTraj, cc->m_robot, *cc->m_brs,
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

/**
 * Generates a plan for moving the arm to a cartesian goal
 */
bool planArmToCartTarget(PlanningProblem& prob, const Eigen::VectorXd& startJoints, const btTransform& goalTrans, RaveRobotObject::Manipulator::Ptr arm) {
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
  CollisionCostPtr cc(new CollisionCost(arm->robot->robot,   arm->robot->getEnvironment()->bullet->dynamicsWorld, brs,
      arm->manip->GetArmIndices(), SQPConfig::distPen, SQPConfig::collCoefInit));
  JointBoundsPtr jb(new JointBounds(true, false, defaultMaxStepMvmt(initTraj) / 5, arm->manip));
  CartesianPoseCostPtr cp(new CartesianPoseCost(arm, goalTrans, initTraj.rows() - 1, 1000., 1000));

  prob.initialize(initTraj, false);
  prob.addComponent(lcc);
  prob.addComponent(cc);
  prob.addTrustRegionAdjuster(jb);
  prob.addComponent(cp);
  return outerOptimization(prob, cc, vector<AllowedLinkCollision>());

}

bool planArmToJointTarget(PlanningProblem& prob, const Eigen::VectorXd& startJoints, const Eigen::VectorXd& endJoints, RaveRobotObject::Manipulator::Ptr arm) {
  BulletRaveSyncherPtr brs = syncherFromArm(arm);
  MatrixXd initTraj = makeTraj(startJoints, endJoints, SQPConfig::nStepsInit); // xxx nsteps
  LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, true, defaultMaxStepMvmt(initTraj), SQPConfig::lengthCoef));
  CollisionCostPtr cc(new CollisionCost(arm->robot->robot,   arm->robot->getEnvironment()->bullet->dynamicsWorld, brs,
      arm->manip->GetArmIndices(), SQPConfig::distPen, SQPConfig::collCoefInit));
  JointBoundsPtr jb(new JointBounds(true, true, defaultMaxStepMvmt(initTraj) / 5, arm->manip));
  prob.initialize(initTraj, true);
  prob.addComponent(lcc);
  prob.addComponent(cc);
  prob.addTrustRegionAdjuster(jb);
  return outerOptimization(prob, cc, vector<AllowedLinkCollision>());
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
  out.middleRows(x0.size(),x1.size());
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
  AllowedLinkCollision allowEndEffector(gripperInd, allowedCollisionIntervals);
  vector<AllowedLinkCollision> allowedCollisions;
  allowedCollisions.push_back(allowEndEffector);

  LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, false, defaultMaxStepMvmt(initTraj), SQPConfig::lengthCoef));
  CollisionCostPtr cc(new CollisionCost(arm->robot->robot,   arm->robot->getEnvironment()->bullet->dynamicsWorld, brs,
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
  CollisionCostPtr cc(new CollisionCost(arm->robot->robot,   arm->robot->getEnvironment()->bullet->dynamicsWorld, brs,
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
  return outerOptimization(prob, cc, vector<AllowedLinkCollision>());
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
