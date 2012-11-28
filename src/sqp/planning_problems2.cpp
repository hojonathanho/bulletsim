#include "planning_problems2.h"
#include "config_sqp.h"
#include "traj_sqp.h"
#include "simulation/openravesupport.h"
#include "kinematics_utils.h"
#include "utils/logging.h"
#include "simulation/bullet_io.h"
#include "collisions.h"

using namespace Eigen;
using namespace std;

static double COLL_COST_MULT = 10;
static double INIT_MAX_DIFF_PER_ITER = .25;
static double OUTER_OPT_ITER_LIMIT = 100;
static double MAX_COLL_COST_RATIO = 1000;

CollisionCostPtr getCollisionCost(TrajOptimizer& opt) {
  BOOST_FOREACH(CostPtr& cost, opt.m_costs) {
    CollisionCostPtr maybeCC = boost::static_pointer_cast<CollisionCost>(cost);
    if (maybeCC) {
      return maybeCC;
    }
  }
  return CollisionCostPtr();
}

vector<int> getManipLinkInds(RobotBase::ManipulatorPtr manip) {
	vector<KinBody::LinkPtr> vlinks;
	manip->GetChildLinks(vlinks);
	vector<int> out;
	BOOST_FOREACH(KinBody::LinkPtr& link, vlinks) {
		if (!link->GetGeometries().empty()) out.push_back(link->GetIndex());
	}
	return out;
}

double calcCollisionScore(const MatrixXd& traj, RaveRobotObject* rro, RobotBase::ManipulatorPtr manip) {
	vector<int> armInds = manip->GetArmIndices();
	TrajCartCollInfo tcci = collectTrajCollisions(traj, rro, armInds, false);
	double out=0;
	BOOST_FOREACH(CartCollInfo& cci, tcci) {
		BOOST_FOREACH(LinkCollision& lc, cci) {
			out += pospart(-lc.dist);
		}
	}
	return out;
}

vector<VectorXd> findCollisionFreeIK(const OpenRAVE::Transform& tf, RaveRobotObject* rro, RobotBase::ManipulatorPtr manip) {
	vector<vector<double> > iksolns;
	vector<int> armInds = manip->GetArmIndices();
	manip->FindIKSolutions(IkParameterization(tf), iksolns, IKFO_IgnoreSelfCollisions | IKFO_IgnoreEndEffectorCollisions);
	MatrixXd traj(iksolns.size(), armInds.size());
	vector<VectorXd> out;
	for (int i=0; i < iksolns.size(); ++i) traj.row(i) = toVectorXd(iksolns[i]);
	TrajCartCollInfo tcci = collectTrajCollisions(traj, rro, armInds, false);
	vector<int> manipLinkInds = getManipLinkInds(manip);
	for (int i = 0; i < iksolns.size(); ++i) {
		CartCollInfo& cci = tcci[i];
		bool safe = true;
		BOOST_FOREACH(LinkCollision& lc, cci) {
			if ((lc.dist < 0) && (find(manipLinkInds.begin(), manipLinkInds.end(), lc.linkInd) == manipLinkInds.end())) {
				string linkName = rro->robot->GetLinks()[lc.linkInd]->GetName();
				LOG_DEBUG("discarding ik solution because link has collision" << linkName);
				safe = false;
				break;
			}
		}
		if (safe) out.push_back(traj.row(i));
	}
	LOG_DEBUG_FMT("found %i ik solns. %i were collision free", iksolns.size(), out.size());
	return out;
}

OuterOptStatus trajOuterOpt(TrajOptimizer& opt, const AllowedCollisions& allowedCollisions) {
  
  CollisionCostPtr cc = getCollisionCost(opt);
  
  OuterOptStatus status = NOTDONE;
  double coll_cost_ratio = 1;
  
  for (int outerOptIter = 0; status == NOTDONE && outerOptIter < OUTER_OPT_ITER_LIMIT; ++outerOptIter) {

    LOG_INFO_FMT("outer optimization iteration: %i", outerOptIter);
    
    if (opt.m_tra->m_shrinkage < SQPConfig::shrinkLimit) {
      LOG_INFO("trajOuterOpt: reducing trust region shrinkage");
      opt.m_tra->adjustTrustRegion(10*SQPConfig::shrinkLimit / opt.m_tra->m_shrinkage);
    }
    
    Optimizer::OptStatus sqpStatus = opt.optimize();
    if (sqpStatus == Optimizer::GRB_FAIL) {
      LOG_INFO("GRB failure in inner optimization");
      status = GRB_FAIL;
      break;
    }

    TrajCartCollInfo discCollInfo = cc->discreteCollisionCheck();
    bool discSafe = isDiscreteSafe(discCollInfo, SQPConfig::distDiscSafe, opt.m_times, allowedCollisions);
        
    
    if (discSafe) {
      LOG_INFO("trajectory is discrete-safe");
      TrajCartCollInfo contCollInfo = cc->continuousCollisionCheck();
      vector<double> insertTimes = getSubdivisionTimes(contCollInfo, opt.m_times, allowedCollisions);
      bool contSafe = insertTimes.size() == 0;
      if (contSafe) {
        LOG_INFO("trajectory is continuous safe");
        status = SUCCESS;
      }
      else {
        LOG_INFO("traj isn't continuous safe. times with collisions: " << insertTimes);
        if (opt.m_times.size() > SQPConfig::maxSteps) {
          LOG_ERROR("max number of steps exceeded");
          status = MAX_STEPS_EXCEEDED;
        }
        else {
          LOG_INFO("subdividing.");
          opt.subdivide(insertTimes);          
        }
      }
    }
    else {
      if (coll_cost_ratio > MAX_COLL_COST_RATIO) {
        LOG_ERROR("maximum (collision) penalty exceeded");
        status = MAX_PENALTY_EXCEEDED;
      }
      else {
        LOG_INFO("trajectory is not discrete-safe. increasing collision penalty");
        cc->m_coeff *= COLL_COST_MULT;
        coll_cost_ratio *= COLL_COST_MULT;              
      }
    }
  }  
  return status;
}

bool setupArmToJointTarget(TrajOptimizer& opt, const VectorXd& endJoints, RobotManipulatorPtr manip) {

  VectorXd lower, upper;
  vector<int> armInds = manip->manip->GetArmIndices();
  getJointLimits(manip->robot->robot, armInds, lower, upper);
  VectorXd maxDiffPerIter = VectorXd::Constant(7, .25);
  VectorXd startJoints = toVectorXd(manip->getDOFValues());

  opt.setTrustRegion(TrustRegionPtr(new JointBounds(&opt, maxDiffPerIter, lower, upper)));
  opt.addCost(CostPtr(new CollisionCost(&opt, manip->robot, armInds, false, SQPConfig::collCoefInit)));
  opt.addCost(CostPtr(new JntLenCost(&opt, SQPConfig::lengthCoef)));
  opt.initialize(linearInterp(startJoints, endJoints, SQPConfig::nStepsInit), arange(SQPConfig::nStepsInit));
  setStartFixed(opt);
  return true;
}

double toNegPiPi1(double x) {
  while (true) {
    if (x > SIMD_PI)
      x -= 2 * SIMD_PI;
    else if (x < -SIMD_PI)
      x += 2 * SIMD_PI;
    else
      break;
  }
  return x;
}

MatrixXd toNegPiPi(const MatrixXd& in) {
  MatrixXd out(in.rows(), in.cols());
  for (int i = 0; i < in.rows(); ++i) {
    for (int j = 0; j < in.cols(); ++j) {
      out(i,j) = toNegPiPi1(in(i,j));
    }
  }
  return out;
}


bool setupArmToFollowCart(TrajOptimizer& opt, const vector<btTransform>& goals, RobotManipulatorPtr arm, KinBody::LinkPtr link) {
  btTransform linkGoal;
  OpenRAVE::Transform eeGoal;

  vector<btTransform> linkGoals;
  BOOST_FOREACH(const btTransform& goal, goals) {
  	btTransform linkGoal;
    if (link) {
    	linkGoal = goal;
      eeGoal = util::toRaveTransform(linkGoal) * link->GetTransform().inverse() * arm->manip->GetEndEffectorTransform();
    }
    else {
      eeGoal = util::toRaveTransform(goal);
      link = arm->manip->GetEndEffector();
      linkGoal = util::toBtTransform(eeGoal * arm->manip->GetLocalToolTransform().inverse());
    }
    linkGoals.push_back(goal);
  }

  VectorXd startJoints = toVectorXd(arm->getDOFValues());

  VectorXd lower, upper;
  vector<int> armInds = arm->manip->GetArmIndices();
  getJointLimits(arm->robot->robot, armInds, lower, upper);
  VectorXd maxDiffPerIter = VectorXd::Constant(armInds.size(), .25);

  VectorXd endJoints = (lower+upper)/2;

  static double POS_COEFF = 100;
  static double ROT_COEFF = 100;

  opt.setTrustRegion(TrustRegionPtr(new JointBounds(&opt, maxDiffPerIter, lower, upper)));
  opt.addCost(CostPtr(new CollisionCost(&opt, arm->robot, armInds, false, SQPConfig::collCoefInit)));
  opt.addCost(CostPtr(new JntLenCost(&opt, SQPConfig::lengthCoef)));
  opt.addCost(CostPtr(new CartPoseCost(&opt, arm->robot->robot, link, armInds, false, linkGoals,
  		VectorXd::Ones(goals.size())*POS_COEFF, VectorXd::Ones(goals.size())*ROT_COEFF, false)));
  opt.initialize(linearInterp(startJoints, endJoints, goals.size()), arange(goals.size()));
  setStartFixed(opt);

  return true;

}

bool setupArmToCartTarget(TrajOptimizer& opt, const btTransform& goal, RobotManipulatorPtr arm, KinBody::LinkPtr link) {

  btTransform linkGoal;
  OpenRAVE::Transform eeGoal;
  if (link) {
    linkGoal = goal;
    // worldFromEE_targ = worldFromLink_targ * (linkFromWorld * worldFromEE)_now
    eeGoal = util::toRaveTransform(linkGoal) * link->GetTransform().inverse() * arm->manip->GetEndEffectorTransform();
  }
  else {
    eeGoal = util::toRaveTransform(goal);
    link = arm->manip->GetEndEffector();
    linkGoal = util::toBtTransform(eeGoal * arm->manip->GetLocalToolTransform().inverse());
  }

  VectorXd startJoints = toVectorXd(arm->getDOFValues());

#define STRAIGHT_LINE_INIT
#ifdef STRAIGHT_LINE_INIT
  vector<VectorXd> safeIkSolns = findCollisionFreeIK(eeGoal, arm->robot, arm->manip);
  bool ikSuccess = safeIkSolns.size() > 0;
  if (!ikSuccess) {
    LOG_ERROR("no collision free ik solution for target! " << eeGoal);
    return false;
  }
  double bestVal=INFINITY;
  int bestInd=-1;
  for (int i=0; i < safeIkSolns.size(); ++i) {
  	MatrixXd traj = linearInterp(startJoints, safeIkSolns[i], 10);
  	double score = calcCollisionScore(traj, arm->robot, arm->manip);
  	LOG_DEBUG_FMT("safe soln %i: score: %.2e", i, score);
  	if (score < bestVal) {
  		bestVal = score;
  		bestInd = i;
  	}
  }
  VectorXd endJoints = safeIkSolns[bestInd];
#else
  VectorXd endJoints = startJoints;
#endif

  VectorXd lower, upper;
  vector<int> armInds = arm->manip->GetArmIndices();
  getJointLimits(arm->robot->robot, armInds, lower, upper);
  VectorXd maxDiffPerIter = VectorXd::Constant(armInds.size(), .25);

  static double POS_COEFF = 1000;
  static double ROT_COEFF = 1000;

  opt.setTrustRegion(TrustRegionPtr(new JointBounds(&opt, maxDiffPerIter, lower, upper)));
  opt.addCost(CostPtr(new CollisionCost(&opt, arm->robot, armInds, false, SQPConfig::collCoefInit)));
  opt.addCost(CostPtr(new JntLenCost(&opt, SQPConfig::lengthCoef)));
  opt.addCost(CostPtr(new CartPoseCost(&opt, arm->robot->robot, link, armInds, false, linkGoal, POS_COEFF, ROT_COEFF, false)));
  opt.initialize(linearInterp(startJoints, endJoints, SQPConfig::nStepsInit), arange(SQPConfig::nStepsInit));
  setStartFixed(opt);
  return true;
}


bool setupArmToGrasp(TrajOptimizer& opt, const btTransform& goal, RobotManipulatorPtr arm) {
  return true;
}


bool setupArmToCartTargetWithBase(TrajOptimizer& opt, const btTransform& goalTrans, RobotManipulatorPtr arm) {
	
	RobotBasePtr robot = arm->robot->robot;

  vector<int> dofInds = arm->manip->GetArmIndices();
  if (robot->GetJoint("torso_lift_joint")) dofInds.push_back(robot->GetJoint("torso_lift_joint")->GetDOFIndex());


	VectorXd startJoints = toVectorXd(arm->robot->getDOFValues(dofInds));
  VectorXd endJoints = startJoints;
	
	OpenRAVE::Transform t = robot->GetTransform();
	float zrot, yrot, xrot;
	util::toBtTransform(t).getBasis().getEulerZYX(zrot, yrot, xrot);
	LOG_INFO("Z,Y,X: " << zrot << " " << yrot << " " << xrot);
	Vector3d startXYA(t.trans.x, t.trans.y, zrot);
  Vector2d goalXY(goalTrans.getOrigin().x(), goalTrans.getOrigin().y());
  Vector2d goalDir = (goalXY - startXYA.topRows(2)).normalized();

	double DIST_BEHIND_GOAL = 1;
	Vector3d endXYA(goalXY(0) - goalDir(0)*DIST_BEHIND_GOAL, goalXY(1) - goalDir(1)*DIST_BEHIND_GOAL, atan2(goalDir(1),goalDir(0)));

	endXYA(2) = startXYA(2);
	VectorXd startState = concatenate(startJoints, startXYA);
	VectorXd endState = concatenate(endJoints, endXYA);
	
	LOG_DEBUG("start state: " << startState.transpose());
	LOG_DEBUG("end state: " << endState.transpose());

  VectorXd lower, upper;
  getJointLimits(arm->robot->robot, dofInds, lower, upper);
  VectorXd maxDiffPerIter = VectorXd::Ones(dofInds.size()+3) * .25;
	
  KinBody::LinkPtr link = arm->manip->GetEndEffector();
  btTransform linkGoal = goalTrans * util::toBtTransform(arm->manip->GetLocalToolTransform()).inverse();
  util::drawSpheres(linkGoal.getOrigin()*METERS, Vector3f(1,0,0), 1, .05*METERS, util::getGlobalEnv());


  static double POS_COEFF = 1000;
  static double ROT_COEFF = 0;
  
  // XXX ZERO ROTATION
  opt.setTrustRegion(TrustRegionPtr(new JointBounds(&opt, maxDiffPerIter, concatenate(lower,Vector3d(-100, -100, 0)), concatenate(upper,Vector3d(100, 100, 0)))));
  opt.addCost(CostPtr(new CollisionCost(&opt, arm->robot, dofInds, true, SQPConfig::collCoefInit)));
  opt.addCost(CostPtr(new JntLenCost(&opt, SQPConfig::lengthCoef)));
  opt.addCost(CostPtr(new CartPoseCost(&opt, arm->robot->robot, link, dofInds, true, linkGoal, POS_COEFF, ROT_COEFF, true)));

  MatrixXd initTraj = linearInterp(startState, endState, SQPConfig::nStepsInit);
  opt.initialize(initTraj, arange(SQPConfig::nStepsInit));
  setStartFixed(opt);
	
  return true;
}
