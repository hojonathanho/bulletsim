
// all lengths are in meters, not bullet scale
void planArmToCartTarget(PlanningProblem& prob, btTransform& target);
void planArmToGrasp(PlanningProblem& prob, btTransform& target);
void planArmToJointTarget(PlanningProblem& prob, Eigen::VectorXd& joints);

bool outerOptimization(PlanningProblem& prob, ...) {
  int outerOptIter=0;
  while (true) {
    LOG_INFO_FMT("multi-resolution optimization iteration: %i", outerOptIter)
    prob.optimize();
    
    // discrete is safe, else double coll coeff (but if it's at the upper limit, quit)
    // continuous is safe, else resample (but if you're at the max number of samples, quit)
    
    if (!isDiscSafe(cc->m_cartCollInfo, distDiscSafe, allowedCollisionIntervals)) { // not discrete safe
      if (cc->m_coeff < maxCollCoeff) {
        cc->m_coeff = fmin(cc->m_coeff*COLL_COST_MULT, maxCollCoeff);
        prob.clearCostHistory();
        LOG_INFO_FMT("trajectory was not discrete-safe. collision coeff <- %.2f", cc->m_coeff);
        continue;
      }
      else {
        LOG_INFO("trajectory was not discrete-safe, but collision coeff is too high! stopping optimization");
        return false;
      }
    }
    
    else { // not continuous safe
      TrajCartCollInfo cci = continuousTrajCollisions(prob.m_currentTraj, pr2->robot, brs, 
          scene.env->bullet->dynamicsWorld, rarm->manip->GetArmIndices(), contSafe);
      if (!isContSafe(cci, distContSafe, allowedCollisionIntervals)) {
        if (prob.traj.rows() < maxSteps) {
          vector<double> insertTimes = getSubdividedTimes(cci, m_times, allowedCollisionIntervals);
          prob.subdivide(insertTimes);
          LOG_INFO("trajectory was discrete-safe but not continuous-safe. subdividing");
          continue;
        }
        else {
          LOG_INFO("trajectory was discrete-safe but not continuous-safe, but it's already too long! stopping optimization");
          return false;
        }
      }
      else {
        LOG_INFO("hooray! trajectory is discrete-safe and continuous safe. stopping optimization")
        return true;
      }
      
    }
    ++outerOptIter;
  }  
}



bool planArmToCartTarget(PlanningProblem& prob, btTransform& target, double distPen, double distDiscSafe, double distContSafe) {  
  
  vector<double> ikSoln;
  bool ikSuccess = arm->solveIKUnscaled(toRaveTransform(goalTrans), ikSoln);
  if !(ikSucces) {
    LOG_ERROR("no ik solution for target!");
    return false;
  }
  VectorXd endJoints = toVectorXd(ikSolns[0]);  
  
	MatrixXd initTraj = makeTraj(startJoints, endJoints, nInitialSteps);
	LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, true, defaultMaxStepMvmt(initTraj),SQPConfig::lengthCoef));
	CollisionCostPtr cc(new CollisionCost(pr2->robot, scene.env->bullet->dynamicsWorld, brs, rarm->manip->GetArmIndices(), -BulletConfig::linkPadding/2, SQPConfig::collCoef));
	JointBoundsPtr jb(new JointBounds(true, true, defaultMaxStepMvmt(initTraj)/5, rarm->manip));
  CartesianPoseCostPtr cp(new CartesianPoseCost(arm, goalTrans, initTraj.rows()-1, 100., 100));
	prob.initialize(initTraj, true);
  prob.addComponent(lcc);
  prob.addComponent(cc);
  prob.addComponent(jb);
  prob.addComponent(cp);  
  return outerOptimization(prob, ...);
      
}

bool planArmToJointTarget(PlanningProblem& prob, btTransform& target, double distPen, double distDiscSafe, double distContSafe) {  
	MatrixXd initTraj = makeTraj(startJoints, endJoints, nInitialSteps);
	LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, true, defaultMaxStepMvmt(initTraj),SQPConfig::lengthCoef));
	CollisionCostPtr cc(new CollisionCost(pr2->robot, scene.env->bullet->dynamicsWorld, brs, rarm->manip->GetArmIndices(), -BulletConfig::linkPadding/2, SQPConfig::collCoef));
	JointBoundsPtr jb(new JointBounds(true, true, defaultMaxStepMvmt(initTraj)/5, rarm->manip));
	prob.initialize(initTraj, true);
  prob.addComponent(lcc);
  prob.addComponent(cc);
  prob.addComponent(jb);
  prob.addComponent(cp);  
  return outerOptimization(prob, ...);
      
}


bool planArmToGrasp(PlanningProblem& prob, btTransform& target) {
  
  vector<double> ikSoln;
  bool ikSuccess = arm->solveIKUnscaled(toRaveTransform(goalTrans), ikSoln);
  if !(ikSucces) {
    LOG_ERROR("no ik solution for target!");
    return false;
  }
  VectorXd endJoints = toVectorXd(ikSolns[0]);  
  
	MatrixXd initTraj = makeTraj(startJoints, endJoints, nInitialSteps);
  addFixedEnd(initTraj, nEnd);
  typedef pair<double,double> paird;
  vector<paird> allowedCollisionIntervals;
  allowedCollisionIntervals.push_back(paird(nInitialSteps, nInitialSteps+nEnd-1));

  // todo: properly subdivide collision coeff

	LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, true, defaultMaxStepMvmt(initTraj),SQPConfig::lengthCoef));
	CollisionCostPtr cc(new CollisionCost(pr2->robot, scene.env->bullet->dynamicsWorld, brs, rarm->manip->GetArmIndices(), -BulletConfig::linkPadding/2, SQPConfig::collCoef));
	JointBoundsPtr jb(new JointBounds(true, true, defaultMaxStepMvmt(initTraj)/5, rarm->manip));
  CartesianPoseCostPtr cp(new CartesianPoseCost(arm, goalTrans, initTraj.rows()-1, 100., 100));	
  CartesianVelConstraintPtr cvc(new CartesianVelConstraint(arm, oldLen, oldLen+nFinal, .02*METERS));
	
	prob.initialize(initTraj, true);
  prob.addComponent(lcc);
  prob.addComponent(cc);
  prob.addComponent(jb);
  prob.addComponent(cp);
  prob.addComponent(cvc);
  
  return outerOptimiation(prob, ...);

}