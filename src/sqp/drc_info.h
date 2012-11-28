#pragma once
#include "sqp/sqp_fwd.h"
#include "simulation/openrave_fwd.h"
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "sqp/expr_ops.h"
using std::string;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
// dynamics-related trajectory optimization / costs

static const double DT1 = .1;

struct DRCInfo {

	static const int NUM_JOINTS = 28;
	static const int NUM_DOF = 35;
	static const int I_POS = 0;
	static const int I_TORQUE = NUM_DOF;
	static const int I_LCOP = NUM_DOF + NUM_JOINTS;
	static const int I_LFORCE = NUM_DOF + NUM_JOINTS + 3;
	static const int I_RCOP = NUM_DOF + NUM_JOINTS + 6;
	static const int I_RFORCE = NUM_DOF + NUM_JOINTS + 9;
  static const int I_END = NUM_DOF + NUM_JOINTS + 12;

	static const int L_FOOT_INDEX = 28;
	static const int R_FOOT_INDEX = 34;

	static VectorXd pos(const MatrixXd& traj, int t) {
		return traj.block(t, I_POS, 1, NUM_DOF).transpose();
	}
	static VectorXd vel(const MatrixXd& traj, int t) {
		return ((t > 0) ? (pos(traj, t) - pos(traj, t-1)) : (pos(traj, 1) - pos(traj, 0)))/DT1;
	}
	static VectorXd acc(const MatrixXd& traj, int t) {
		return ((t > 0) ? (vel(traj, t) - vel(traj, t-1)) : (vel(traj, 1) - vel(traj, 0)))/DT1;
	}
	static VectorXd torque(const MatrixXd& traj, int t) {
		return traj.block(t, I_TORQUE, 1, NUM_JOINTS).transpose();
	}
	static Vector3d leftCop(const MatrixXd& traj, int t) {
		return traj.block(t, I_LCOP, 1, 3).transpose();
	}
	static Vector3d leftForce(const MatrixXd& traj, int t) {
		return traj.block(t, I_LFORCE, 1, 3).transpose();
	}
	static Vector3d rightCop(const MatrixXd& traj, int t) {
		return traj.block(t, I_RCOP, 1, 3).transpose();
	}
	static Vector3d rightForce(const MatrixXd& traj, int t) {
		return traj.block(t, I_RFORCE, 1, 3).transpose();
	}

	static VarVector posSym(const VarArray& traj, int t) {
		return traj.rblock(t, I_POS, NUM_DOF);
	}
	static ExprVector velSym(const VarArray& traj, int t) {
		return exprMult((t > 0) ? exprSub(posSym(traj, t),posSym(traj, t-1)) : exprSub(posSym(traj, 1),posSym(traj, 0)), 1./DT1);
	}
	static ExprVector accSym(const VarArray& traj, int t) {
		return exprMult((t > 0) ? exprSub(velSym(traj, t),velSym(traj, t-1)) : exprSub(velSym(traj, 1),velSym(traj, 0)), 1./DT1);
	}
	static VarVector torqueSym(const VarArray& traj, int t) {
		return traj.rblock(t, I_TORQUE, NUM_JOINTS);
	}
	static VarVector leftCopSym(const VarArray& traj, int t) {
		return traj.rblock(t, I_LCOP, 3);
	}
	static VarVector leftForceSym(const VarArray& traj, int t) {
		return traj.rblock(t, I_LFORCE, 3);
	}
	static VarVector rightCopSym(const VarArray& traj, int t) {
		return traj.rblock(t, I_RCOP, 3);
	}
	static VarVector rightForceSym(const VarArray& traj, int t) {
		return traj.rblock(t, I_RFORCE, 3);
	}

	static VectorXd dynamicsVec(const MatrixXd& traj, int t);
	static ExprVector dynamicsVecSym(const VarArray& traj, int t);


};


void setAllDofsActive(RobotBasePtr robot);
