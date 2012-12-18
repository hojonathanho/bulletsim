#include "ilqr_solver.h"
#include "eigen_io_util.h"

double cost_goal(const VectorXd& b, const VectorXd& goal_mu) {
	VectorXd mu; MatrixXd rt_Sigma;
	parse_belief_state(b, mu, rt_Sigma);
	double cost =  (mu - goal_mu).squaredNorm() + (rt_Sigma * rt_Sigma.transpose()).trace();
	return 10*cost;
}

VectorXd dcost_goal(const VectorXd& b, const VectorXd& goal_mu) {
	VectorXd J = 2*b;
	J.segment(0,goal_mu.rows()) = J.segment(0, goal_mu.rows()) - 2 * goal_mu;
	return 10*J;
}

MatrixXd Hcost_goal(const VectorXd& b, const VectorXd& goal_mu) {
	MatrixXd H = 2*MatrixXd::Identity(b.rows(), b.rows());
	return 10*H;
}

double cost_time(const VectorXd& b, const VectorXd& u, const VectorXd& b_bar, const VectorXd& u_bar) {
	double cost = 0.001*u.squaredNorm(); //+ (b-b_bar).squaredNorm() + (u-u_bar).squaredNorm();
	return cost;
}

void dcost_time(const VectorXd& b, const VectorXd& u, const VectorXd& b_bar, const VectorXd& u_bar,
		VectorXd& Jb, VectorXd& Ju) {
	Jb = VectorXd::Zero(b.rows()); //b - 2*b_bar;
	Ju = 2*0.001*u; //+ u - 2*u_bar;
}

void Hcost_time(const VectorXd& b, const VectorXd& u, const VectorXd& b_bar, const VectorXd& u_bar,
		MatrixXd& Hb, MatrixXd& Hu, MatrixXd& Pbu) {
	Hb = MatrixXd::Zero(b.rows(), b.rows()); //MatrixXd::Identity(b.rows(), b.rows());
	Hu = 2*0.001*MatrixXd::Identity(u.rows(), u.rows());// + MatrixXd::Identity(u.rows(), u.rows());
	Pbu = MatrixXd::Zero(u.rows(), b.rows());
}


void ilqr_solver(Robot &r, const vector<VectorXd>& B_bar,
		const vector<VectorXd>& U_bar, const vector<MatrixXd>& W_bar,
		const double rho_x, const double rho_u, const VectorXd goal_mu,
		const int N_iter, vector<VectorXd>& opt_X, vector<VectorXd>& opt_U,
		vector<MatrixXd>& K, vector<VectorXd>& u0) {

	  int NX = B_bar[0].rows();
	  int NU = U_bar[0].rows();
	  int T = U_bar.size();
	  int NS = W_bar[0].cols();
	  assert(T+1 == B_bar.size());

	  vector<VectorXd> B_ilqr(T+1), U_ilqr(T);
	  for (int i = 0; i < T+1; i++)
	    B_ilqr[i] = B_bar[i];
	  for (int i = 0; i < T; i++)
	    U_ilqr[i] = U_bar[i];

	  opt_X = vector<VectorXd>(T+1);
	  opt_U = vector<VectorXd>(T);
	  double lastCost = 999999;
	  double eps = 0.1;

	  for (int iter = 0; iter < N_iter; iter++) {
		  cout << "iter: " << iter << endl;
		  vector<MatrixXd> L(T);
		  vector<VectorXd> l(T);
		  MatrixXd S = Hcost_goal(B_ilqr[T], goal_mu);
		  VectorXd s_b = dcost_goal(B_ilqr[T], goal_mu);
		  double s = cost_goal(B_ilqr[T], goal_mu);
		  for (int t = T-1; t >= 0; t--) {
			  MatrixXd F, G;
			  r.dbdb(B_ilqr[t], U_ilqr[t], F);
			  r.dbdu(B_ilqr[t], U_ilqr[t], G);
			  vector<MatrixXd> F_i(NX), G_i(NX);
			  vector<VectorXd> e_b_i(NX);
			  for (int i = 0; i < NX; i++) {
				  r.dbndb(B_ilqr[t], U_ilqr[t], i, F_i[i]);
				  r.dbndu(B_ilqr[t], U_ilqr[t], i, G_i[i]);
				  MatrixXd W;
				  r.belief_noise(B_ilqr[t], U_ilqr[t], W);
				  e_b_i[i] = W.col(i);
			  }

			  MatrixXd Q, R, P;
			  VectorXd q_b, r_b;
			  double p;
			  Hcost_time(B_ilqr[t], U_ilqr[t], B_ilqr[t], U_ilqr[t], Q, R, P);
			  dcost_time(B_ilqr[t], U_ilqr[t], B_ilqr[t], U_ilqr[t], q_b, r_b);
			  //Hcost_time(B_ilqr[t], U_ilqr[t], B_bar[t], U_bar[t], Q, R, P);
			  //dcost_time(B_ilqr[t], U_ilqr[t], B_bar[t], U_bar[t], q_b, r_b);
			  p = cost_time(B_ilqr[t], U_ilqr[t], B_ilqr[t], U_ilqr[t]);
			  //p = cost_time(B_ilqr[t], U_ilqr[t], B_bar[t], U_bar[t]);


			  MatrixXd C, D, E;
			  VectorXd c_b, d_b;
			  double e;
			  C = Q + F.transpose()*S*F;
			  D = R + G.transpose()*S*G;
			  E = P + G.transpose()*S*F;
			  c_b = q_b + F.transpose()*s_b;
			  d_b = r_b + G.transpose()*s_b;
			  e = p + s;

			  for (int i = 0; i < NX; i++) {
				  C += F_i[i].transpose() * S * F_i[i];
				  D += G_i[i].transpose() * S * G_i[i];
				  E += G_i[i].transpose() * S * F_i[i];
				  c_b += F_i[i].transpose() * S * e_b_i[i];
				  d_b += G_i[i].transpose() * S * e_b_i[i];
				  e += (0.5)*e_b_i[i].transpose() * S * e_b_i[i];
			  }



			  PartialPivLU<MatrixXd> solver(D);
			  L[t] = solver.solve(-E);
			  l[t] = solver.solve(-d_b);

			  S = C + E.transpose()*L[t];
			  s_b = c_b + E.transpose()*l[t];
			  s = e + 0.5*d_b.transpose()*l[t];

		  }

		opt_X[0] = B_bar[0];
		for (int t = 0; t < T; t++) {
			opt_U[t] = L[t] * (opt_X[t] - B_ilqr[t]) + eps * l[t] + U_ilqr[t];
			r.belief_dynamics(opt_X[t], opt_U[t], opt_X[t + 1]);
		}
		//double currentCost = computeExpectedCost(r, opt_X, opt_U, L, goal_mu);
		//cout << "lastCost: " << lastCost << ", currentCost: " << currentCost		<< endl;
		B_ilqr = opt_X;
		U_ilqr = opt_U;
		K = L;
		u0 = l;
//		if (currentCost < lastCost) {
//			B_ilqr = opt_X;
//			U_ilqr = opt_U;
//			lastCost = currentCost;
//			eps = 0.1;
//		} else {
//			eps /= 2.0;
//		}

	  }
}

double computeExpectedCost(Robot &r, const vector<VectorXd>& B_ilqr,
		const vector<VectorXd>& U_ilqr, const vector<MatrixXd>& L, const VectorXd& goal_mu) {
	double cost = 0;
	int NX = B_ilqr[0].rows();
	int NU = U_ilqr[0].rows();
	int T = U_ilqr.size();

	MatrixXd S = Hcost_goal(B_ilqr[T], goal_mu);
	VectorXd s_b = dcost_goal(B_ilqr[T], goal_mu);
	double s = cost_goal(B_ilqr[T], goal_mu);
	for (int t = T - 1; t >= 0; t--) {
		  MatrixXd F, G;
		  r.dbdb(B_ilqr[t], U_ilqr[t], F);
		  r.dbdu(B_ilqr[t], U_ilqr[t], G);
		  vector<MatrixXd> F_i(NX), G_i(NX);
		  vector<VectorXd> e_b_i(NX);
		  for (int i = 0; i < NX; i++) {
			  r.dbndb(B_ilqr[t], U_ilqr[t], i, F_i[i]);
			  r.dbndu(B_ilqr[t], U_ilqr[t], i, G_i[i]);
			  MatrixXd W;
			  r.belief_noise(B_ilqr[t], U_ilqr[t], W);
			  e_b_i[i] = W.col(i);
		  }

		  MatrixXd Q, R, P;
		  VectorXd q_b, r_b;
		  double p;
		  Hcost_time(B_ilqr[t], U_ilqr[t], B_ilqr[t], U_ilqr[t], Q, R, P);
		  dcost_time(B_ilqr[t], U_ilqr[t], B_ilqr[t], U_ilqr[t], q_b, r_b);
		  p = cost_time(B_ilqr[t], U_ilqr[t], B_ilqr[t], U_ilqr[t]);

		  MatrixXd L_t = L[t];

		  MatrixXd S_t1;
		  VectorXd s_b_t1;
		  double s_t1;
		  S_t1 = Q + L_t.transpose()*R*L_t + L_t.transpose()*P + P.transpose()*L_t;
		  MatrixXd FGL_t = F * G * L_t;
		  S_t1 += (FGL_t).transpose()*S*(FGL_t);
		  s_b_t1 = q_b + L_t.transpose()*r_b + (F + G*L_t).transpose()*s_b;
		  s_t1 = p + s_t1;
		  for (int i = 0; i < NX; i++) {
			  S_t1 += (F_i[i] + G_i[i]*L_t).transpose()*S*(F_i[i] + G_i[i]*L_t);
			  s_b_t1 += (F_i[i] + G_i[i]*L_t).transpose()*S*e_b_i[i];
			  s_t1 += 0.5*e_b_i[i].transpose()*S*e_b_i[i];
		  }
		  S = S_t1;
		  s_b = s_b_t1;
		  s = s_t1;
	}
	return s;
}
