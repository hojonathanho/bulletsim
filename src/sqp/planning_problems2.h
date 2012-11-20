#pragma once
#include "sqp/sqp_fwd.h"
#include "simulation/simulation_fwd.h"
#include <LinearMath/btTransform.h>
#include <Eigen/Dense>
#include "traj_safety_checks.h"
#include <openrave/openrave.h>

using Eigen::VectorXd;

enum OuterOptStatus {
  NOTDONE,
  SUCCESS,
  GRB_FAIL,
  MAX_STEPS_EXCEEDED,
  MAX_PENALTY_EXCEEDED,
  ITERATION_LIMIT
};

OuterOptStatus trajOuterOpt(TrajOptimizer& opt, const AllowedCollisions&);

bool setupArmToJointTarget(TrajOptimizer& opt, const VectorXd& endJoints,  RobotManipulatorPtr manip);
bool setupArmToCartTarget(TrajOptimizer& opt, const btTransform& goal, RobotManipulatorPtr arm, KinBody::LinkPtr link=KinBody::LinkPtr());
bool setupArmToGrasp(TrajOptimizer& opt, const btTransform& goal, RobotManipulatorPtr arm);
bool setupArmToCartTargetWithBase(TrajOptimizer& opt, const btTransform& goalTrans, RobotManipulatorPtr arm);
bool setupArmToFollowCart(TrajOptimizer& opt, const vector<btTransform>& goals, RobotManipulatorPtr arm, KinBody::LinkPtr link);
