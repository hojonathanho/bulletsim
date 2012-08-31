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
    sensors[i]->observe(sensor_fns[i](x), z_i);
    z.segment(num_recv, z_i.rows()) = z_i;
    num_recv += z_i.rows();
  }
}

void Robot::attach_sensor(Sensor* sensor, SensorFunc f) {
  sensors.push_back(sensor);
  sensor_fns.push_back(f);
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
void Robot::dp_trajectory(const vector<VectorXd>& X_bar, vector<MatrixXd>& Ps, vector<VectorXd>& p_offsets) {

  int T = X_bar.size();
  Ps.resize(T); p_offsets.resize(T);
  for (int t = 0; t < T; t++) {
    MatrixXd P; VectorXd p_offset;
    dpdx(X_bar[t], P, p_offset);
    Ps[t] = P; p_offsets[t] = p_offset;
  }
}

void Robot::dbp_trajectory(const vector<VectorXd>& B_bar, vector<MatrixXd>& Ps, vector<VectorXd>& p_offsets) {
	int T = B_bar.size();
	int NB = B_bar[0].rows();
	Ps.resize(T); p_offsets.resize(T);
	MatrixXd Ps_tmp = MatrixXd::Zero(0, NB);
	VectorXd p_offsets_tmp = VectorXd::Zero(0);
	for (int t = 0; t < T; t++) {
		Ps[t] = Ps_tmp;
		p_offsets[t] = p_offsets_tmp;
	}
}


void Robot::dgdx(const VectorXd& x, MatrixXd& C) {
  C = MatrixXd::Zero(_NZ, _NX);
  for (int i = 0; i < _NX; i++) {
    VectorXd eps_vec = VectorXd::Zero(_NX);
    eps_vec(i) = _eps;
    VectorXd x_pos = x + eps_vec;
    VectorXd x_neg = x - eps_vec;
    VectorXd gx_pos(_NX); VectorXd gx_neg(_NX); 
    observe(x_pos, gx_pos);
    observe(x_neg, gx_neg);
    C.col(i) = (gx_pos - gx_neg) / (2*_eps); 
  }
}

void Robot::dg(const VectorXd& x, MatrixXd& C, VectorXd& d){
  // approximates g(x) = Cx + d where d = g(xb)-C*xb
  // xb is the linearization point
  dgdx(x,C); 
  observe(x, d);
  d = d - C*x; 
}

void Robot::dbdb(const VectorXd &b, const VectorXd &u, MatrixXd &A) {
  A = MatrixXd::Zero(_NB, _NB);
  for (int i = 0; i < _NB; i++) {
    VectorXd eps_vec = VectorXd::Zero(_NB);
    eps_vec(i) = _eps; 
    VectorXd b_pos = b + eps_vec;
    VectorXd b_neg = b - eps_vec;
    VectorXd bb_pos(_NB); VectorXd bb_neg(_NB); 
    belief_dynamics(b_pos, u, bb_pos);
    belief_dynamics(b_neg, u, bb_neg);
    A.col(i) = (bb_pos - bb_neg) / (2*_eps); 
  }
}

void Robot::dbdu(const VectorXd &b, const VectorXd &u, MatrixXd &B) {
  B = MatrixXd::Zero(_NB, _NU);
  for (int i = 0; i < _NU; i++) {
    VectorXd eps_vec = VectorXd::Zero(_NU);
    eps_vec(i) = _eps; 
    VectorXd u_pos = u + eps_vec;
    VectorXd u_neg = u - eps_vec;
    VectorXd bu_pos(_NB); VectorXd bu_neg(_NB); 
    belief_dynamics(b, u_pos, bu_pos);
    belief_dynamics(b, u_neg, bu_neg);
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

