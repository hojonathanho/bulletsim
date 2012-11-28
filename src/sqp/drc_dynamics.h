#pragma once
#include "sqp/sqp_fwd.h"
#include "simulation/openrave_fwd.h"
#include <Eigen/Dense>
#include "sqp/utils_sqp.h"
#include "sqp/functions.h"
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;

struct DynamicsErrorCalculator: public fVectorOfVector {
	RobotBasePtr m_robot;
	vector<MatrixXd> m_jacs;
	DynamicsErrorCalculator(RobotBasePtr robot);
	VectorXd operator()(const VectorXd& in) const;
	VectorXd calcErr(const vector<MatrixXd>& jacs, const VectorXd& p,
			const VectorXd& v, const VectorXd& a, const VectorXd& y,
			const Vector3d& leftCop, const Vector3d& leftForce,
			const Vector3d& rightCop, const Vector3d& rightForce) const;
};

ExprVector makeDynamicsErrorExpr(const ExprVector& xvars, const VectorXd& x,
		RobotBasePtr robot);
