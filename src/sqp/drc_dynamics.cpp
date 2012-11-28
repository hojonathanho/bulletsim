#include "sqp/drc_costs.h"
#include "sqp/drc_dynamics.h"
#include <openrave/openrave.h>
#include <Eigen/Dense>
#include <vector>
#include "sqp/functions.h"
#include "sqp/expr_ops.h"
#include <boost/foreach.hpp>
#include "sqp/utils_sqp.h"
#include "utils/logging.h"
#include "utils/utils_vector.h"


using namespace OpenRAVE;
using namespace Eigen;
using namespace std;

static const double GRAV_ACC = 9.8;

inline OpenRAVE::Vector toRaveVector(const Vector3d& v) {
	return OpenRAVE::Vector(v[0], v[1], v[2]);
}

void setAllDofsActive(RobotBasePtr robot) {
	vector<int> activeDofs;
	for (int i = 0; i < robot->GetDOF(); ++i)
		activeDofs.push_back(i);
	robot->SetActiveDOFs(activeDofs, OpenRAVE::DOF_Transform);
}

DynamicsErrorCalculator::DynamicsErrorCalculator(RobotBasePtr robot) :
		m_robot(robot) {
	BOOST_FOREACH(const KinBody::LinkPtr& link, robot->GetLinks()) {
		vector<double> jac;
		robot->CalculateActiveJacobian(link->GetIndex(), link->GetTransform().trans, jac);
		assert (jac.size() == 3*DRCInfo::NUM_DOF);
		m_jacs.push_back(Map<MatrixXd>(jac.data(), 3, DRCInfo::NUM_DOF));
	}
}

VectorXd DynamicsErrorCalculator::operator()(const VectorXd& in) const {
	int numDof = DRCInfo::NUM_DOF;
	VectorXd y(numDof);
	y.topRows(numDof - 7) = in.middleRows(3 * numDof, numDof - 7);
	y.bottomRows(7).setZero();

	return calcErr(m_jacs,
	in.middleRows(0, numDof), // p
	in.middleRows(numDof, numDof), // v
	in.middleRows(2 * numDof, numDof), //a
	y,
	in.middleRows(4 * numDof - 7 + 0, 3),
	in.middleRows(4 * numDof - 7 + 3, 3),
	in.middleRows(4 * numDof - 7 + 6, 3),
	in.middleRows(4 * numDof - 7 + 9, 3));
}

VectorXd DynamicsErrorCalculator::calcErr(const vector<MatrixXd>& jacs,
		const VectorXd& p, const VectorXd& v, const VectorXd& a, const VectorXd& y,
		const Vector3d& leftCop, const Vector3d& leftForce,
		const Vector3d& rightCop, const Vector3d& rightForce) const {

	m_robot->SetDOFValues(toDoubleVec(p), false);
	int numLinks = jacs.size();
	int numDof = p.size();
	VectorXd err = VectorXd::Zero(numDof);

	// link energy. TODO: rotational energy
	const vector<KinBody::LinkPtr>& links = m_robot->GetLinks();
	for (int iLink = 0; iLink < numLinks; ++iLink) {
		double mass = links[iLink]->GetMass();
		err += mass * jacs[iLink].transpose() * (jacs[iLink] * a);
		err -= mass * GRAV_ACC * jacs[iLink].row(2);
	}

	// energy terms due to foot forces
	vector<double> footjac;
	m_robot->CalculateActiveJacobian(DRCInfo::L_FOOT_INDEX, toRaveVector(leftCop),
			footjac);
	err += leftForce.transpose() * Map<MatrixXd>(footjac.data(), 3, numDof);
	m_robot->CalculateActiveJacobian(DRCInfo::R_FOOT_INDEX, toRaveVector(rightCop),
			footjac);
	err += rightForce.transpose() * Map<MatrixXd>(footjac.data(), 3, numDof);
	// energy terms due to torque
	err -= y;

	return err;
}

ExprVector makeDynamicsErrorExpr(const ExprVector& xvars, const VectorXd& x,
		RobotBasePtr robot) {
	DynamicsErrorCalculator errCalc(robot);
	MatrixXd errJac = calcJacobian(errCalc, x, 1e-6);
	ExprVector out = exprMatMult(errJac, xvars);
	exprInc(out, errCalc(x) - errJac * x);
	return out;
}

