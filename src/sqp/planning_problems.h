#pragma once
#include "sqp/sqp_fwd.h"
#include <LinearMath/btTransform.h>
#include <Eigen/Dense>
#include "simulation/openravesupport.h"

bool planArmToJointTarget(PlanningProblem& prob, const Eigen::VectorXd& startJoints,
    const Eigen::VectorXd& endJoints, RaveRobotObject::Manipulator::Ptr arm, bool doOptimize);
bool planArmToCartTarget(PlanningProblem& prob, const Eigen::VectorXd& startJoints,
    const btTransform& endJoints, RaveRobotObject::Manipulator::Ptr arm, bool doOptimize=true);
bool planArmToGrasp(PlanningProblem& prob, const Eigen::VectorXd& startJoints,
    const btTransform& goalTrans, RaveRobotObject::Manipulator::Ptr arm);
bool planArmBaseToCartTarget(PlanningProblem& prob, const Eigen::VectorXd& startJoints, const btTransform& goalTrans,
                             RaveRobotObject::Manipulator::Ptr arm);
