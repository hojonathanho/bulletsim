#include "sqp/drc_costs.h"
#include "sqp/utils_sqp.h"
#include "drc_dynamics.h"
#include "drc_info.h"

using namespace Eigen;
using namespace OpenRAVE;

inline OpenRAVE::Vector toRaveVector(const Vector3d& v) {
	return OpenRAVE::Vector(v[0], v[1], v[2]);
}
inline Vector3d toVector3d(const OpenRAVE::Vector& v) {
	return Vector3d(v.x, v.y, v.z);
}

EulerLagrangeCost::EulerLagrangeCost(TrajOptimizer* opt, RobotBasePtr robot, double coeff) :
TrajCost(opt),
m_robot(robot),
m_coeff(coeff)
{}

double EulerLagrangeCost::evaluate(const MatrixXd& traj) {
	double out = 0;
	for (int iStep=0; iStep < getLength(); ++iStep) {
		m_robot->SetActiveDOFValues(toDoubleVec(traj.row(iStep)));
		DynamicsErrorCalculator errCalc(m_robot);
		VectorXd dynVec = DRCInfo::dynamicsVec(traj, iStep);
		out += m_coeff * errCalc(dynVec).squaredNorm();
	}
	return out;
}

ConvexObjectivePtr EulerLagrangeCost::convexify(GRBModel* model) {
	ConvexObjectivePtr out(new ConvexObjective());
	for (int iStep=0; iStep < getLength(); ++iStep) {
		m_robot->SetActiveDOFValues(toDoubleVec(getTraj().row(iStep)));
		VectorXd dynVec = DRCInfo::dynamicsVec(getTraj(), iStep);
		ExprVector dynVecSym = DRCInfo::dynamicsVecSym(getVars(), iStep);
		ExprVector errVec = makeDynamicsErrorExpr(dynVecSym, dynVec, m_robot);
		for (int i=0; i < errVec.size(); ++i) {
			out->m_objective += m_coeff * (errVec[i] * errVec[i]);
//			addAbsCost(out, m_coeff, errVec[i], model, "el_err");
		}
	}
	return out;
}


StayAboveGround::StayAboveGround(TrajOptimizer* opt, RobotBasePtr robot, double coeff) :
		TrajConstraint(opt),
		m_robot(robot),
		m_coeff(coeff)
		{}

double StayAboveGround::getViolVal(const MatrixXd& traj) {
  double out = 0;
  for (int iStep=0; iStep < getLength(); ++iStep) {
  	m_robot->SetDOFValues(toDoubleVec(DRCInfo::pos(traj, iStep)));
  	out += pospart(-DRCInfo::leftCop(traj, iStep)[2]);
  	out += pospart(-DRCInfo::rightCop(traj, iStep)[2]);
  }
  return out;
}

ConvexConstraintPtr StayAboveGround::convexify(GRBModel* model) {
  ConvexConstraintPtr out(new ConvexConstraint());
  MatrixXd& traj = getTraj();
  VarArray& vars = getVars();
  for (int iStep=0; iStep < getLength(); ++iStep) {
  	m_robot->SetDOFValues(toDoubleVec(DRCInfo::pos(traj, iStep)));

  	out->m_eqcnts.push_back(model->addConstr(DRCInfo::leftCopSym(vars, iStep)[2] >= 0));
  	out->m_eqcnts.push_back(model->addConstr(DRCInfo::rightCopSym(vars, iStep)[2] >= 0));


  	VectorXd jointsCur = DRCInfo::pos(traj, iStep);
  	VarVector jointsSym = DRCInfo::posSym(vars, iStep);

  	ExprVector lFootErr(3);
  	Vector3d lFootCur = toVector3d(m_robot->GetLinks()[DRCInfo::L_FOOT_INDEX]->GetTransform().trans);
  	Vector3d lCopCur = DRCInfo::leftCop(traj, iStep);
  	VarVector lCopSym = DRCInfo::leftCopSym(vars, iStep);
  	vector<double> lJacVec;
  	m_robot->CalculateActiveJacobian(DRCInfo::L_FOOT_INDEX, toRaveVector(lCopCur), lJacVec);
  	MatrixXd lJac = Map<MatrixXd>(lJacVec.data(), 3, DRCInfo::NUM_DOF);
  	exprInc(lFootErr, lFootCur - lCopCur);
  	exprInc(lFootErr, exprSub(exprMatMult(lJac, jointsSym), lJac * jointsCur));
  	exprDec(lFootErr, exprSub(lCopSym, lCopCur));
  	for (int i=0; i < 3; ++i) {
  		out->m_eqcnts.push_back(model->addConstr(lFootErr[i] == 0));
  	}

  	ExprVector rFootErr(3);
  	Vector3d rFootCur = toVector3d(m_robot->GetLinks()[DRCInfo::R_FOOT_INDEX]->GetTransform().trans);
  	Vector3d rCopCur = DRCInfo::rightCop(traj, iStep);
  	VarVector rCopSym = DRCInfo::rightCopSym(vars, iStep);
  	vector<double> rJacVec;
  	m_robot->CalculateActiveJacobian(DRCInfo::R_FOOT_INDEX, toRaveVector(rCopCur), rJacVec);
  	MatrixXd rJac = Map<MatrixXd>(rJacVec.data(), 3, DRCInfo::NUM_DOF);
  	exprInc(rFootErr, rFootCur - rCopCur);
  	exprInc(rFootErr, exprSub(exprMatMult(rJac, jointsSym), rJac * jointsCur));
  	exprDec(rFootErr, exprSub(rCopSym, rCopCur));
  	for (int i=0; i < 3; ++i) {
  		out->m_eqcnts.push_back(model->addConstr(rFootErr[i] == 0));
  	}


  }
  return out;
}
