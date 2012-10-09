#include "planning_problems.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "utils/vector_io.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "sqp_algorithm.h"
#include "config_sqp.h"
#include "sqp/config_sqp.h"


using namespace std;
using namespace Eigen;

typedef std::pair<double,double> paird;


static double COLL_COST_MULT = 3;

bool inSomeInterval(double d, const std::vector<paird>& exclude) {
  bool excluded = false;
  BOOST_FOREACH(const paird& interval, exclude) {
    if (d >= interval.first && d <= interval.second) excluded=true;
  }
  return excluded;
}

vector<double> filterOutIntervals(const std::vector<double>& in, const std::vector<paird>& exclude) {
  vector<double> out;
  BOOST_FOREACH(const double d, in) if (!inSomeInterval(d, exclude)) out.push_back(d);
  return out;
}




bool isSafe(const TrajCartCollInfo& cci, double distSafe, const Eigen::VectorXd& times, const std::vector<paird>& allowedCollisionIntervals) {
//  countCollisions(const TrajJointCollInfo& trajCollInfo, double safeDist, int& nNear, int& nUnsafe, int& nColl)
  assert(cci.size() == times.size());
  vector<double> discreteUnsafeTimes;
  for (int i=0; i < cci.size(); ++i) {
    if (!inSomeInterval(times(i), allowedCollisionIntervals)) {
      BOOST_FOREACH(const LinkCollision& lc, cci[i]) {
        if (lc.dist < distSafe) {
          discreteUnsafeTimes.push_back(times(i));
        }
      }
    }
  }
  if (discreteUnsafeTimes.size() >0) {
    LOG_INFO("steps with a discrete collision: " << discreteUnsafeTimes);
    stringstream ss;
    BOOST_FOREACH(paird p, allowedCollisionIntervals) ss << "(" << p.first << "," << p.second << ") ";
    LOG_INFO("allowed collision intervals:" << ss.str());
  }
  return discreteUnsafeTimes.size()==0;
}


vector<double> getSubdivisionTimes(const TrajCartCollInfo& cci, const Eigen::VectorXd& times) {
  int nContColl=0;
  vector<double> insertTimes;
  for (int i = 0; i < cci.size(); ++i) {
    nContColl += cci[i].size();
    if (cci[i].size() > 0) insertTimes.push_back((times(i) + times(i+1)) / 2);
  }
  return insertTimes;
}


bool outerOptimization(PlanningProblem& prob, CollisionCostPtr cc, const std::vector<paird>& allowedCollisionIntervals, double maxCollCoeff, int maxSteps) {
  for (int outerOptIter = 0;; ++outerOptIter) {
    LOG_INFO_FMT("outer optimization iteration: %i", outerOptIter);
    prob.optimize(SQPConfig::maxIter);
    printAllConstraints(*prob.m_model);

    // discrete is safe, else double coll coeff (but if it's at the upper limit, quit)
    // continuous is safe, else resample (but if you're at the max number of samples, quit)

    TrajCartCollInfo& discCollInfo = cc->m_cartCollInfo;


    if (!isSafe(discCollInfo, SQPConfig::distDiscSafe, prob.m_times,allowedCollisionIntervals)) { // not discrete safe
      if (cc->m_coeff < maxCollCoeff) {
        cc->m_coeff = fmin(cc->m_coeff * COLL_COST_MULT, maxCollCoeff);
        LOG_INFO_FMT("trajectory was not discrete-safe. collision coeff <- %.2f", cc->m_coeff);
        continue;
      }
      else {
        LOG_INFO("trajectory was not discrete-safe, but collision coeff is too high! stopping optimization");
        return false;
      }
    }
    else { // not continuous safe
      TrajCartCollInfo contCollInfo = continuousTrajCollisions(prob.m_currentTraj, cc->m_robot, cc->m_brs,
          cc->m_world, cc->m_dofInds, SQPConfig::distContSafe);
      vector<double> insertTimes = getSubdivisionTimes(contCollInfo, prob.m_times);
      insertTimes = filterOutIntervals(insertTimes, allowedCollisionIntervals);
      if (insertTimes.size() > 0) {
        LOG_INFO("trajectory was discrete-safe but not continuous-safe");
        if (prob.m_times.size() < maxSteps) {
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


bool planArmToCartTarget(PlanningProblem& prob, const Eigen::VectorXd& startJoints, const btTransform& goalTrans, RaveRobotObject::Manipulator::Ptr arm) {
  BulletRaveSyncher brs = syncherFromArm(arm);
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
  return outerOptimization(prob, cc, vector<paird>(), 200, 100);

}

bool planArmToJointTarget(PlanningProblem& prob, const Eigen::VectorXd& startJoints, const Eigen::VectorXd& endJoints, RaveRobotObject::Manipulator::Ptr arm) {
  BulletRaveSyncher brs = syncherFromArm(arm);
  MatrixXd initTraj = makeTraj(startJoints, endJoints, SQPConfig::nStepsInit); // xxx nsteps
  LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, true, defaultMaxStepMvmt(initTraj), SQPConfig::lengthCoef));
  CollisionCostPtr cc(new CollisionCost(arm->robot->robot,   arm->robot->getEnvironment()->bullet->dynamicsWorld, brs,
      arm->manip->GetArmIndices(), SQPConfig::distPen, SQPConfig::collCoefInit));
  JointBoundsPtr jb(new JointBounds(true, true, defaultMaxStepMvmt(initTraj) / 5, arm->manip));
  prob.initialize(initTraj, true);
  prob.addComponent(lcc);
  prob.addComponent(cc);
  prob.addTrustRegionAdjuster(jb);
  return outerOptimization(prob, cc, vector<paird>(), 200, 100);
}

void addFixedEnd(MatrixXd& traj, int nEnd) {
  int oldLen = traj.rows();
  traj.conservativeResize(oldLen + nEnd, NoChange);
  for (int i=0; i < nEnd; ++i) traj.row(oldLen+i) = traj.row(oldLen-1);
}

bool planArmToGrasp(PlanningProblem& prob, const Eigen::VectorXd& startJoints, const btTransform& goalTrans, RaveRobotObject::Manipulator::Ptr arm) {
  BulletRaveSyncher brs = syncherFromArm(arm);
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
  cout << "old len " << oldLen << endl;
  addFixedEnd(initTraj, nEnd);
  cout << "new len " << initTraj.rows() << endl;
  typedef pair<double, double> paird;
  vector<paird> allowedCollisionIntervals;
  allowedCollisionIntervals.push_back(paird(oldLen, oldLen+nEnd));


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
  return outerOptimization(prob, cc, allowedCollisionIntervals, 200, 100);

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
