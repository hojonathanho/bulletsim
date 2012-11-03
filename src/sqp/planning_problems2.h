#pragma once
#include "sqp/sqp_fwd.h"
#include "simulation/simulation_fwd.h"
#include <LinearMath/btTransform.h>
#include <Eigen/Dense>
using Eigen::VectorXd;

void setupArmToJointTarget(TrajOptimizer& opt, const VectorXd& endJoints);
void setupArmToCartTarget(TrajOptimizer& opt, const btTransform& goal, RobotManipulatorPtr arm);
void setupArmToGrasp(TrajOptimizer& opt, const btTransform& goal, RobotManipulatorPtr arm);
void setupArmToCartTargetWithBase(TrajOptimizer& opt, const btTransform& goalTrans, RobotManipulatorPtr arm);
