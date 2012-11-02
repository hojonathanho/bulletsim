#pragma once
#include "sqp/sqp_fwd.h"
#include "simulation/simulation_fwd.h"
#include <LinearMath/btTransform.h>
#include <Eigen/Dense>
using Eigen::VectorXd;

void setupArmToJointTarget(Optimizer& opt, const VectorXd& endJoints);
void setupArmToCartTarget(Optimizer& opt, const btTransform& goal, RobotManipulatorPtr arm);
void setupArmToGrasp(Optimizer& opt, const btTransform& goal, RobotManipulatorPtr arm);
void setupArmToCartTargetWithBase(Optimizer& opt, const btTransform& goalTrans, RobotManipulatorPtr arm);
