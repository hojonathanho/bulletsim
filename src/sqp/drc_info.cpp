#include "sqp/drc_info.h"

VectorXd DRCInfo::dynamicsVec(const MatrixXd& traj, int t) {
	VectorXd out(3 * NUM_DOF + NUM_JOINTS + 12);
	out.middleRows(0, NUM_DOF) = pos(traj, t);
	out.middleRows(NUM_DOF, NUM_DOF) = vel(traj, t);
	out.middleRows(2 * NUM_DOF, NUM_DOF) = acc(traj, t);
	out.middleRows(3 * NUM_DOF, NUM_JOINTS) = torque(traj, t);
	out.middleRows(3 * NUM_DOF + NUM_JOINTS, 3) = leftCop(traj, t);
	out.middleRows(3 * NUM_DOF + NUM_JOINTS + 3, 3) = leftForce(traj, t);
	out.middleRows(3 * NUM_DOF + NUM_JOINTS + 6, 3) = rightCop(traj, t);
	out.middleRows(3 * NUM_DOF + NUM_JOINTS + 9, 3) = rightForce(traj, t);
	return out;
}
ExprVector DRCInfo::dynamicsVecSym(const VarArray& traj, int t) {
	ExprVector out(3 * NUM_DOF + NUM_JOINTS + 12);
	VarVector p = posSym(traj, t);
	for (int i = 0; i < NUM_DOF; ++i)
		out[i] = p[i];
	ExprVector v = velSym(traj, t);
	for (int i = 0; i < NUM_DOF; ++i)
		out[i + NUM_DOF] = v[i];
	ExprVector a = accSym(traj, t);
	for (int i = 0; i < NUM_DOF; ++i)
		out[i + 2 * NUM_DOF] = a[i];
	VarVector tq = torqueSym(traj, t);
	for (int i = 0; i < NUM_JOINTS; ++i)
		out[i + 3 * NUM_DOF] = tq[i];
	VarVector vv = leftCopSym(traj, t);
	for (int i = 0; i < 3; ++i)
		out[i + 3 * NUM_DOF + NUM_JOINTS] = vv[i];
	vv = leftForceSym(traj, t);
	for (int i = 0; i < 3; ++i)
		out[i + 3 * NUM_DOF + NUM_JOINTS + 3] = vv[i];
	vv = rightCopSym(traj, t);
	for (int i = 0; i < 3; ++i)
		out[i + 3 * NUM_DOF + NUM_JOINTS + 6] = vv[i];
	vv = rightForceSym(traj, t);
	for (int i = 0; i < 3; ++i)
		out[i + 3 * NUM_DOF + NUM_JOINTS + 9] = vv[i];
	return out;
}
