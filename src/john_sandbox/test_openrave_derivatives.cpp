
#include <openrave-core.h>
#include <iostream>
using namespace OpenRAVE;
using namespace std;


void CalculateActiveNumericalJacobian(RobotBasePtr robot, int link_ind, const Vector& offset, double epsilon, vector<double>& out) {
	int numdof = robot->GetActiveDOF();
	vector<double> dofvals;
	robot->GetActiveDOFValues(dofvals);
	assert(dofvals.size() == numdof);
	vector<double> pert_dofvals = dofvals;

	KinBody::LinkPtr link = robot->GetLinks()[link_ind];
	Transform link_trans = link->GetTransform();
	Vector offset_local = link_trans.inverse() * offset;

	out.resize(3*numdof);
	for (int idof=0; idof < numdof; ++idof) {
		pert_dofvals[idof] = dofvals[idof] + epsilon;
		robot->SetActiveDOFValues(pert_dofvals, false);
		Transform pert_link_trans = link->GetTransform();
		Vector pert_offset = pert_link_trans * offset_local;
		for (int j=0; j < 3; ++j) {
			out[numdof*j + idof] = (pert_offset[j] - offset[j])/epsilon;
		}
		pert_dofvals[idof] = dofvals[idof];
	}

	robot->SetActiveDOFValues(dofvals);

}


float randf() {return (float)rand()/(float)RAND_MAX;}

int main(int argc, char* argv[]) {

	// parameters
	DOFAffine affine = DOF_Rotation3D;
	Vector offset(randf(), randf(), randf());
	const double ABS_TOL = 1e-5;
	const double EPSILON = 1e-6;
	/////////////


	// setup
	RaveInitialize(false);
	EnvironmentBasePtr env = RaveCreateEnvironment();
	env->Load("robots/pr2-beta-static.zae");
	vector<RobotBasePtr> robots;
	env->GetRobots(robots);
	RobotBasePtr robot = robots[0];
	vector<int> active_dofs;
	for (int i=0; i < robot->GetDOF(); ++i) active_dofs.push_back(i);
	robot->SetActiveDOFs(active_dofs, affine, Vector(0,0,1));
	int numdof = robot->GetActiveDOF();
	//////////

	// set dofs to random values
	vector<double> lower_limits, upper_limits;
	robot->GetDOFLimits(lower_limits, upper_limits);
	vector<double> dofvals;
	for (int i=0; i < robot->GetDOF(); ++i) dofvals.push_back( lower_limits[i] + randf() * (upper_limits[i] - lower_limits[i]));
	robot->SetDOFValues(dofvals);
//	Transform tf;
//	tf.trans = Vector(0,0,0);
//	tf.rot = Vector(0.18257419,  0.36514837,  0.54772256,  0.73029674);
//	robot->SetTransform(tf);
	/////////////////



	for (int link_ind=0; link_ind < robot->GetLinks().size(); ++link_ind) {
		printf("checking link %i\n", link_ind);
		vector<double> jac;
		robot->CalculateActiveJacobian(link_ind, offset, jac);

		vector<double> numerical_jac;
		CalculateActiveNumericalJacobian(robot, link_ind, offset, EPSILON, numerical_jac);

		assert(jac.size() == numerical_jac.size());

		for (int j=0; j < 3; ++j) {
			for (int idof = 0; idof < numdof; ++idof) {
				int vecind = j*numdof + idof;
				double abserr = fabs(numerical_jac[vecind] - jac[vecind]);
				if (abserr > ABS_TOL) {
					printf("dof %2i | output %2i | analytic %10.3e | numerical: %10.3e\n", idof, j, jac[vecind], numerical_jac[vecind]);
				}
			}
		}


	}


}
