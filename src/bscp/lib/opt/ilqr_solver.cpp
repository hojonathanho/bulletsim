#include "ilqr_solver.h"

double cost_goal(VectorXd& b, VectorXd& goal_mu) {
	VectorXd mu; MatrixXd rt_Sigma;
	parse_belief_state(b, mu, rt_Sigma);
	double cost =  (mu - goal_mu).transpose() * (mu - goal_mu) + (rt_Sigma * rt_Sigma.transpose()).trace();
	return cost;
}

VectorXd dcost_goal(VectorXd& b, VectorXd& goal_mu) {
	VectorXd J = VectorXd::Zero(b.rows());
	J.segment(0,goal_mu.rows()) = - 2 * goal_mu;
	return J;
}

MatrixXd Hcost_goal(VectorXd& b, VectorXd& goal_mu) {
	MatrixXd H = MatrixXd::Zero(b.rows(), b.rows());

}

void ilqr_solver(Robot &r, const vector<VectorXd>& X_bar,
		const vector<VectorXd>& U_bar, const vector<MatrixXd>& W_bar,
		const double rho_x, const double rho_u, VectorXd goal_mu,
		const int N_iter, vector<VectorXd>& opt_X, vector<VectorXd>& opt_U,
		MatrixXd& K, VectorXd& u0) {


}
