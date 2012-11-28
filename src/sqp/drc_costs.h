#pragma once
#include "sqp/drc_info.h"
#include "simulation/openrave_fwd.h"
#include "sqp/traj_sqp.h"

struct EulerLagrangeCost : public TrajCost {
	RobotBasePtr m_robot;
	double m_coeff;
	EulerLagrangeCost(TrajOptimizer* opt, RobotBasePtr robot, double coeff);
	double evaluate(const MatrixXd& traj);
	ConvexObjectivePtr convexify(GRBModel* model);
};

struct StayAboveGround : public TrajConstraint {
	RobotBasePtr m_robot;
	double m_coeff;
	StayAboveGround(TrajOptimizer* opt, RobotBasePtr robot, double coeff);
	double getViolVal(const MatrixXd& traj);
	ConvexConstraintPtr convexify(GRBModel* model);
};
