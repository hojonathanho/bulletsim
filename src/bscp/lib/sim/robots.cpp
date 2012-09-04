#include "robots.h"
#include "kalman_filter.h"
#include "eigen_io_util.h"

void Robot::greet() { 
  cout << "How's it going?" << endl;
}

void Robot::forward_integrate(const VectorXd& x0, const vector<VectorXd>& u, const vector<VectorXd>& w, vector<VectorXd>& traj_x) {
	int T = u.size();
	bool propagate_beliefs = false;
	if (x0.rows() == _NB) propagate_beliefs = true;
	traj_x.resize(T+1);
	traj_x[0] = x0;
	for (int t = 0; t < T; t++) {
		if(propagate_beliefs) {
			belief_dynamics(traj_x[t], u[t], traj_x[t+1]);
		} else {
			dynamics(traj_x[t], u[t], traj_x[t+1]);
		}
		traj_x[t+1] += w[t];
	}
}


void Robot::observe(const VectorXd& x, VectorXd& z) {
  z.resize(_NZ);
  int num_recv = 0; 
  for (int i = 0; i < sensors.size(); i++) {
    VectorXd z_i;
    sensors[i]->observe(sensor_fns[i](*this, x), z_i);
    z.segment(num_recv, z_i.rows()) = z_i;
    num_recv += z_i.rows();
  }
}

void Robot::attach_sensor(Sensor* sensor, SensorFunc g_i, SensorFuncJacobian dg_i) {
  sensors.push_back(sensor);
  sensor_fns.push_back(g_i);
  sensor_fns_jac.push_back(dg_i);
  _NZ += sensor->NZ(); 
}

void Robot::belief_dynamics(const VectorXd& b, const VectorXd& u, VectorXd& bt1) {

  VectorXd x; MatrixXd rt_Sigma;
  parse_belief_state(b, x, rt_Sigma);

  MatrixXd M_t, N_t;
  M(x,M_t);
  N(x,N_t); 
  
  VectorXd x_t1; MatrixXd rt_Sigma_t1; 
  ekf_update(*this,x,u,rt_Sigma,VectorXd::Zero(0),M_t,N_t,
      x_t1, rt_Sigma_t1, false);

  build_belief_state(x_t1, rt_Sigma_t1, bt1);

}

void Robot::belief_dynamics_fixed_model(const VectorXd& b, const VectorXd& u, const MatrixXd& A, const MatrixXd& C, VectorXd& bt1) {
	VectorXd x; MatrixXd rt_Sigma;
	parse_belief_state(b, x, rt_Sigma);

	MatrixXd M_t, N_t;
	M(x, M_t);
	N(x, N_t);

	VectorXd x_t1;
	MatrixXd rt_Sigma_t1;
	ekf_update_fixed_model(*this, x, u, rt_Sigma, VectorXd::Zero(0), A, C, M_t, N_t, x_t1,
			rt_Sigma_t1, false);

	build_belief_state(x_t1, rt_Sigma_t1, bt1);

}

// note the belief noise comes in the form L where L*L' = W
void Robot::belief_noise(const VectorXd& b, const VectorXd& u, MatrixXd& rt_W) {
	VectorXd x; MatrixXd rt_Sigma;
	parse_belief_state(b, x, rt_Sigma);

	MatrixXd M_t, N_t;
	M(x, M_t);
	N(x, N_t);

	MatrixXd rt_W_x;
	ekf_mean_noise_model(*this, x, u, rt_Sigma, M_t, N_t, rt_W_x);

	rt_W = MatrixXd::Zero(_NB, _NB);
	rt_W.block(0,0,_NX,_NX) = rt_W_x;


}

void Robot::dfdx(const VectorXd &x, const VectorXd &u, MatrixXd &A) {
  A = MatrixXd::Zero(_NX, _NX);
  for (int i = 0; i < _NX; i++) {
    VectorXd eps_vec = VectorXd::Zero(_NX);
    eps_vec(i) = _eps; 
    VectorXd x_pos = x + eps_vec;
    VectorXd x_neg = x - eps_vec;
    VectorXd fx_pos(_NX); VectorXd fx_neg(_NX); 
    dynamics(x_pos, u, fx_pos);
    dynamics(x_neg, u, fx_neg);
    A.col(i) = (fx_pos - fx_neg) / (2*_eps); 
  }
}

void Robot::dfdu(const VectorXd &x, const VectorXd &u, MatrixXd &B) {
  B = MatrixXd::Zero(_NX, _NU);
  for (int i = 0; i < _NU; i++) {
    VectorXd eps_vec = VectorXd::Zero(_NU);
    eps_vec(i) = _eps; 
    VectorXd u_pos = u + eps_vec;
    VectorXd u_neg = u - eps_vec;
    VectorXd fu_pos(_NX); VectorXd fu_neg(_NX); 
    dynamics(x, u_pos, fu_pos);
    dynamics(x, u_neg, fu_neg);
    B.col(i) = (fu_pos - fu_neg) / (2*_eps); 
  }
}

void Robot::df(const VectorXd &x, const VectorXd &u, MatrixXd &A, MatrixXd &B, VectorXd &c) {
  // approximates f(x,u) = Ax + Bu + c where c = f(xb,ub) - A*xb - B*xb
  // here xb is the linearization point 
  dfdx(x,u,A);
  dfdu(x,u,B);
  dynamics(x,u,c);
  c = c - A*x - B*u;
}

void Robot::df_trajectory(const vector<VectorXd>& X_bar, const vector<VectorXd>& U_bar, vector<MatrixXd>& As, vector<MatrixXd>& Bs, vector<VectorXd>& Cs) {

  int T = U_bar.size();
  As.resize(T); Bs.resize(T); Cs.resize(T); 
  for (int t = 0; t < T; t++) {
    MatrixXd A; MatrixXd B; VectorXd C; 
    df(X_bar[t], U_bar[t], A, B, C); 
    As[t] = A; Bs[t] = B; Cs[t] = C; 
  }
}

void Robot::dpdx(const VectorXd& x, MatrixXd& P){
	VectorXd px;
	penetration(x, px);
	P = MatrixXd::Zero(px.rows(), _NX);
	for (int i = 0; i < _NX; i++) {
		VectorXd eps_vec = VectorXd::Zero(_NX);
		eps_vec(i) = _eps;
		VectorXd x_pos = x + eps_vec;
		VectorXd x_neg = x - eps_vec;
		VectorXd px_pos(_NX);
		VectorXd px_neg(_NX);
		penetration(x_pos, px_pos);
		penetration(x_neg, px_neg);
		P.col(i) = (px_pos - px_neg) / (2 * _eps);
	}
}

void Robot::dp(const VectorXd& x, MatrixXd& P, VectorXd& p_offset) {
	dpdx(x,P);
	penetration(x,p_offset);
	p_offset = p_offset - P*x;
}

void Robot::dp_trajectory(const vector<VectorXd>& X_bar, vector<MatrixXd>& Ps, vector<VectorXd>& p_offsets) {

  int T = X_bar.size();
  Ps.resize(T); p_offsets.resize(T);
  for (int t = 0; t < T; t++) {
    MatrixXd P; VectorXd p_offset;
    dp(X_bar[t], P, p_offset);
    Ps[t] = P; p_offsets[t] = p_offset;
  }
}


void Robot::bpenetration(const VectorXd& b, VectorXd& p) {
	VectorXd mu_x; MatrixXd rt_cov_x;
	parse_belief_state(b, mu_x, rt_cov_x);

	vector<VectorXd> pos_samples(_NX), neg_samples(_NX);
	for (int i = 0 ; i < _NX; i++) {
		pos_samples[i] = mu_x + rt_cov_x.col(i);
		neg_samples[i] = mu_x - rt_cov_x.col(i);
	}

	vector<VectorXd> pos_transform(_NX), neg_transform(_NX);
	for (int i = 0; i < _NX; i++) {
		penetration(pos_samples[i], pos_transform[i]);
		penetration(neg_samples[i], neg_transform[i]);
	}

	p = VectorXd::Zero(pos_transform[0].rows());
	for (int i = 0; i < _NX; i++ ) {
		p += pos_transform[i];
		p += neg_transform[i];
	}

	p /= (_NX*2.0);
}

//void Robot::dbpdx(const VectorXd &b, MatrixXd& P) {
//	VectorXd mu_x; MatrixXd rt_Sigma_x;
//	parse_belief_state(b, mu_x, rt_Sigma_x);
//	VectorXd px;
//	penetration(mu_x, px);
//	P = MatrixXd::Zero(px.rows(), _NB);
//	for (int i = 0; i < _NB; i++) {
//		VectorXd eps_vec = VectorXd::Zero(_NB);
//		eps_vec(i) = _eps;
//		VectorXd b_pos = b + eps_vec;
//		VectorXd b_neg = b - eps_vec;
//		VectorXd pb_pos(_NB);
//		VectorXd pb_neg(_NB);
//		bpenetration(b_pos, pb_pos);
//		bpenetration(b_neg, pb_neg);
//		P.col(i) = (pb_pos - pb_neg) / (2 * _eps);
//	}
//}

void Robot::dbpdx(const VectorXd &b, MatrixXd& P) {
	VectorXd mu_x; MatrixXd rt_cov_x;
	parse_belief_state(b, mu_x, rt_cov_x);

	vector<VectorXd> pos_samples(_NX), neg_samples(_NX);
	for (int i = 0 ; i < _NX; i++) {
		pos_samples[i] = mu_x + rt_cov_x.col(i);
		neg_samples[i] = mu_x - rt_cov_x.col(i);
	}
	vector<MatrixXd> pos_transform(_NX), neg_transform(_NX);
	for (int i = 0; i < _NX; i++) {
		dpdx(pos_samples[i], pos_transform[i]);
		dpdx(neg_samples[i], neg_transform[i]);
	}

	P = MatrixXd::Zero(pos_transform[0].rows(), _NB);

	int ind = 0;

	for (int i = 0; i < _NX; i++) {
		MatrixXd P_sub = MatrixXd::Zero(pos_transform[i].rows(), pos_transform[i].cols());
		P_sub += pos_transform[i];
		P_sub -= neg_transform[i];
		P_sub /= (_NX * 2.0);
		P.block(0,_NX+ind, P_sub.rows(), _NX - i) = P_sub.block(0,i,P_sub.rows(), _NX-i);
		ind += _NX - i;
	}

	MatrixXd P_sub = MatrixXd::Zero(pos_transform[0].rows(), _NX);
	for (int i = 0; i < _NX; i++) {
		P_sub += pos_transform[i];
		P_sub += neg_transform[i];
	}
	P_sub /= (_NX * 2.0);
	P.block(0,0,P_sub.rows(), _NX) = P_sub;

}

void Robot::dbp(const VectorXd& b, MatrixXd& P, VectorXd& p_offset) {
	dbpdx(b,P);
	bpenetration(b,p_offset);
	p_offset = p_offset - P*b;
}


void Robot::dbp_trajectory(const vector<VectorXd>& B_bar, vector<MatrixXd>& Ps, vector<VectorXd>& p_offsets) {
	int T = B_bar.size();
	Ps.resize(T);
	p_offsets.resize(T);
	for (int t = 0; t < T; t++) {
		MatrixXd P; VectorXd p_offset;
		dbp(B_bar[t], P, p_offset);
		Ps[t] = P;
		p_offsets[t] = p_offset;
	}
}


//void Robot::dgdx(const VectorXd& x, MatrixXd& C) {
//  C = MatrixXd::Zero(_NZ, _NX);
//  for (int i = 0; i < _NX; i++) {
//    VectorXd eps_vec = VectorXd::Zero(_NX);
//    eps_vec(i) = _eps;
//    VectorXd x_pos = x + eps_vec;
//    VectorXd x_neg = x - eps_vec;
//    VectorXd gx_pos(_NX); VectorXd gx_neg(_NX);
//    observe(x_pos, gx_pos);
//    observe(x_neg, gx_neg);
//    C.col(i) = (gx_pos - gx_neg) / (2*_eps);
//  }
//}

void Robot::dg(const VectorXd& x, MatrixXd& C, VectorXd& d){
  // approximates g(x) = Cx + d where d = g(xb)-C*xb
  // xb is the linearization point
  dgdx(x,C);
  observe(x, d);
  d = d - C*x; 
}

void Robot::dgdx(const VectorXd& x, MatrixXd& C) {
	C = MatrixXd::Zero(_NZ, _NX);
	int num_recv = 0;
	for (int i = 0; i < sensors.size(); i++) {
		MatrixXd T;
		if (sensor_fns_jac[i] == NULL) {
			dtdx(x, sensor_fns[i], T);
		} else {
			T = sensor_fns_jac[i](*this, x);
		}
		MatrixXd G_i;
		sensors[i]->dgdx(sensor_fns[i](*this, x), _eps, G_i);
		C.block(num_recv,0,G_i.rows(),_NX) = G_i * T;
	}
}

void Robot::dtdx(const VectorXd &x, SensorFunc& f, MatrixXd& T) {
	int NT = f(*this, x).rows();
	T = MatrixXd::Zero(NT, _NX);
	for (int i = 0; i < _NX; i++) {
		VectorXd eps_vec = VectorXd::Zero(_NX);
		eps_vec(i) = _eps;
		VectorXd x_pos = x + eps_vec;
		VectorXd x_neg = x - eps_vec;
		VectorXd t_pos = f(*this, x_pos);
		VectorXd t_neg = f(*this, x_neg);
		T.col(i) = (t_pos - t_neg) / (2 * _eps);
	}
}

//void Robot::dbdb(const VectorXd &b, const VectorXd &u, MatrixXd &A) {
//  A = MatrixXd::Zero(_NB, _NB);
//  for (int i = 0; i < _NB; i++) {
//    VectorXd eps_vec = VectorXd::Zero(_NB);
//    eps_vec(i) = _eps;
//    VectorXd b_pos = b + eps_vec;
//    VectorXd b_neg = b - eps_vec;
//    VectorXd bb_pos(_NB); VectorXd bb_neg(_NB);
//    belief_dynamics(b_pos, u, bb_pos);
//    belief_dynamics(b_neg, u, bb_neg);
//    A.col(i) = (bb_pos - bb_neg) / (2*_eps);
//  }
//}

void Robot::dbdb(const VectorXd &b, const VectorXd& u, MatrixXd& A) {
	A = MatrixXd::Zero(_NB, _NB);
	for (int i = 0; i < _NX; i++) {
		VectorXd eps_vec = VectorXd::Zero(_NB);
		eps_vec(i) = _eps;
		VectorXd b_pos = b + eps_vec;
		VectorXd b_neg = b - eps_vec;
		VectorXd bb_pos(_NB); VectorXd bb_neg(_NB);
		belief_dynamics(b_pos, u, bb_pos);
		belief_dynamics(b_neg, u, bb_neg);
		A.col(i) = (bb_pos - bb_neg) / (2 * _eps);
	}

	VectorXd x; MatrixXd rt_Sigma;
	parse_belief_state(b, x, rt_Sigma);
	MatrixXd A_x; MatrixXd H_x;
	dfdx(x, u, A_x);
	dgdx(x, H_x);

	for (int i = _NX; i < _NB; i++) {
		VectorXd eps_vec = VectorXd::Zero(_NB);
		eps_vec(i) = _eps;
		VectorXd b_pos = b + eps_vec;
		VectorXd b_neg = b - eps_vec;
		VectorXd bb_pos(_NB); VectorXd bb_neg(_NB);
		belief_dynamics_fixed_model(b_pos, u, A_x, H_x, bb_pos);
		belief_dynamics_fixed_model(b_neg, u, A_x, H_x, bb_neg);
		A.col(i) = (bb_pos - bb_neg) / (2 * _eps);
	}
}

void Robot::dbdu(const VectorXd &b, const VectorXd &u, MatrixXd &B) {
  B = MatrixXd::Zero(_NB, _NU);
  VectorXd x; MatrixXd rt_Sigma;
  parse_belief_state(b, x, rt_Sigma);
  MatrixXd H_x; // sensor model is fixed for u
  dgdx(x, H_x);
  for (int i = 0; i < _NU; i++) {
    VectorXd eps_vec = VectorXd::Zero(_NU);
    eps_vec(i) = _eps; 
    VectorXd u_pos = u + eps_vec;
    VectorXd u_neg = u - eps_vec;
	MatrixXd A_pos, A_neg;
	dfdx(x, u_pos, A_pos); dfdx(x, u_neg, A_neg);
    VectorXd bu_pos(_NB); VectorXd bu_neg(_NB); 
    belief_dynamics_fixed_model(b, u_pos, A_pos, H_x, bu_pos);
    belief_dynamics_fixed_model(b, u_neg, A_neg, H_x, bu_neg);
    B.col(i) = (bu_pos - bu_neg) / (2*_eps); 
  }
}

void Robot::db(const VectorXd &b, const VectorXd &u, MatrixXd &A, MatrixXd &B, VectorXd &c) {
  // approximates f(x,u) = Ax + Bu + c where c = f(xb,ub) - A*xb - B*xb
  // here xb is the linearization point 
  
  dbdb(b,u,A);
  dbdu(b,u,B);
  belief_dynamics(b,u,c);
  c = c - A*b - B*u;
}


void Robot::db_trajectory(const vector<VectorXd>& B_bar, const vector<VectorXd>& U_bar, vector<MatrixXd>& As, vector<MatrixXd>& Bs, vector<VectorXd>& Cs) {
  int T = U_bar.size();
  As.resize(T); Bs.resize(T); Cs.resize(T); 
  for (int t = 0; t < T; t++) {
    MatrixXd A; MatrixXd B; VectorXd C; 
    db(B_bar[t], U_bar[t], A, B, C); 
    As[t] = A; Bs[t] = B; Cs[t] = C; 
  }
}

void Robot::dxyz(const VectorXd & x, MatrixXd& Jxyz) {
	Jxyz = MatrixXd::Zero(3, _NX);
	for (int i = 0; i < _NX; i++) {
		VectorXd eps_vec = VectorXd::Zero(_NX);
		eps_vec(i) = _eps;
		VectorXd x_pos = x + eps_vec;
		VectorXd x_neg = x - eps_vec;
		VectorXd xyz_pos = xyz(x_pos);
		VectorXd xyz_neg = xyz(x_neg);
		Jxyz.col(i) = (xyz_pos - xyz_neg) / (2 * _eps);
	}
}


vector<osg::Node*> Robot::draw_trajectory(vector<VectorXd> &traj_x, Vector4d color, osg::Group* parent) {
  int T = traj_x.size();
  vector<osg::Node*> render;

  for (int t = 0; t < T; t++) {
    render.push_back(draw(traj_x[t], color, parent));
  }
  return render;
}

vector<osg::Node*> Robot::draw_belief_trajectory(vector<VectorXd> &traj_b, Vector4d mean_color, Vector4d ellipsoid_color, osg::Group* parent, double z_offset) {
  int T = traj_b.size();
  vector<osg::Node*> render;

  for (int t = 0; t < T; t++) {
    render.push_back(draw_belief(traj_b[t], mean_color, ellipsoid_color, parent, z_offset));
  }
  return render;
}

void Robot::unscented_transform_xyz(const VectorXd& mu_x, const MatrixXd& cov_x,
		   Vector3d& mu_y, Matrix3d& Sigma_y) {
	int NX = mu_x.rows();
	int NY = 3;
	double L = NX;
	double h = 3.0 - L;
	LLT<MatrixXd> lltOfCovX((L+h) * cov_x);
	MatrixXd sqrt_mat = lltOfCovX.matrixL();
	vector<VectorXd> pos_samples(L), neg_samples(L);
	vector<double> weights(L);
	for (int i =0 ; i < L; i++) {
		pos_samples[i] = mu_x + sqrt_mat.col(i);
		neg_samples[i] = mu_x - sqrt_mat.col(i);
		weights[i] = 1.0 / (2.0 * (L + h));
	}

	vector<Vector3d> pos_transform(L), neg_transform(L);
	for (int i = 0; i < L; i++) {
		pos_transform[i] = xyz(pos_samples[i]);
		neg_transform[i] = xyz(neg_samples[i]);
	}

	mu_y = (h / (L + h)) * xyz(mu_x);
	for (int i = 0; i < L; i++ ) {
		mu_y += weights[i] * pos_transform[i];
		mu_y += weights[i] * neg_transform[i];
	}

	Sigma_y = 2.0 * (xyz(mu_x) - mu_y) * ((xyz(mu_x) - mu_y).transpose());

	for (int i = 0; i < L; i++) {
		Vector3d diff_pos = pos_transform[i] - mu_y;
		Vector3d diff_neg = neg_transform[i] - mu_y;
		Sigma_y += weights[i] * diff_pos * diff_pos.transpose();
		Sigma_y += weights[i] * diff_neg * diff_neg.transpose();
	}


}

