#ifndef _localizer_h
#define _localizer_h

#include "robots.h"

using namespace std; 
using namespace Eigen;


class Localizer : public Robot {
  public:
	Robot* _r;
	int _num_obj;
    MatrixXd _M; bool M_set;
    MatrixXd _N; bool N_set;

   Localizer(Robot* r, int num_obj) :
	   Robot(r->_NX + 3*num_obj,r->_NU,r->_eps,r->_dt)
   {
	   _r = r;
	   _num_obj = num_obj;
	   M_set = false;
	   N_set = false;
   }

   inline void parse_localizer_state(const VectorXd& X, VectorXd& X_r, VectorXd& X_o) {
	   X_r = X.segment(0, _r->_NX);
	   X_o = X.segment(_r->_NX, 3*_num_obj);
   }

   inline void parse_localizer_belief_state(const VectorXd& b, VectorXd& b_r) {
	   VectorXd x; MatrixXd rt_Sigma;
	   parse_belief_state(b, x, rt_Sigma);
	   VectorXd x_r, x_o;
	   parse_localizer_state(x, x_r, x_o);
	   MatrixXd rt_Sigma_r;
	   rt_Sigma_r = rt_Sigma.block(0,0,_r->_NX, _r->_NX);
	   build_belief_state(x_r, rt_Sigma_r, b_r);
   }

   void dynamics(const VectorXd &x, const VectorXd &u, VectorXd &fxu) {
	   VectorXd X_r, X_o;
	   parse_localizer_state(x, X_r, X_o);
	   VectorXd fxu_r;
	   _r->dynamics(X_r, u, fxu_r);
	   fxu = VectorXd(_NX);
	   fxu.segment(0,_r->_NX) = fxu_r;
	   fxu.segment(_r->_NX, 3*_num_obj) = X_o;
   }

   void penetration(const VectorXd& x, VectorXd& p) {
	   VectorXd X_r, X_o;
	   parse_localizer_state(x, X_r, X_o);
	   _r->penetration(X_r, p);
   }

   double x_upper_limit(const int index) {
	   if (index < _r->_NX)
		   return _r->x_upper_limit(index);
	   else
		   return 1e20;
   }

   double x_lower_limit(const int index) {
	   if (index < _r->_NX)
		   return _r->x_lower_limit(index);
	   else
		   return -1e20;
   }

   double u_upper_limit(const int index) {
	   return _r->u_upper_limit(index);
   }
   double u_lower_limit(const int index) {
	   return _r->u_lower_limit(index);
   }

   void M(const VectorXd& x, MatrixXd& M) {
      assert(M_set == true);
      M = _M;
    }

    void N(const VectorXd& x, MatrixXd& N) {
      assert(N_set == true);
      N = _N;
    }

    void set_M(const MatrixXd& M) {
      _M = M;
      M_set = true;
    }

    void set_N(const MatrixXd& N) {
      _N = N;
      N_set = true;
    }

   osg::Node* draw(VectorXd x, Vector4d color, osg::Group* parent) {
	   VectorXd X_r, X_o;
	   parse_localizer_state(x, X_r, X_o);
	   return _r->draw(X_r, color, parent);
   }

   osg::Node* draw_belief(VectorXd b, Vector4d mean_color, Vector4d ellipsoid_color, osg::Group* parent, double z_offset=0) {
	   VectorXd b_r;
	   parse_localizer_belief_state(b, b_r);
	   return _r->draw_belief(b_r, mean_color, ellipsoid_color, parent, z_offset);
   }

   Vector3d xyz(const VectorXd &x) {
	   VectorXd x_r, x_o;
	   parse_localizer_state(x, x_r, x_o);
	   return _r->xyz(x_r);
   }
   
   Vector4d quat(const VectorXd& x) {
	   VectorXd x_r, x_o;
	   parse_localizer_state(x, x_r, x_o);
	   return _r->quat(x_r);
   }

   void observe(const VectorXd& x, VectorXd& z) {
	   VectorXd x_r, x_o;
	   parse_localizer_state(x, x_r, x_o);
	   VectorXd z_self, z_r;
	   Robot::observe(x, z_self);
	   _r->observe(x_r, z_r);
	   z = VectorXd::Zero(_NZ + _r->_NZ);
	   z.segment(0,_NZ) = z_self;
	   z.segment(_NZ, _r->_NZ) = z_r;
   }


   // dynamics linearization
   void dfdx(const VectorXd &x, const VectorXd &u, MatrixXd &A) {
	   VectorXd x_r, x_o;
	   parse_localizer_state(x, x_r, x_o);
	   A = MatrixXd::Zero(_NX, _NX);
	   MatrixXd A_r;
	   _r->dfdx(x_r, u, A_r);
	   A.block(0,0,_r->_NX,_r->_NX) = A_r;
   }
   void dfdu(const VectorXd &x, const VectorXd &u, MatrixXd &B) {
	   VectorXd x_r, x_o;
	   parse_localizer_state(x, x_r, x_o);
	   B = MatrixXd::Zero(_NX, _NU);
	   MatrixXd B_r;
	   _r->dfdu(x_r, u, B_r);
	   B.block(0,0,_r->_NX,_NU) = B_r;
   }

     //observation linearization
   void dgdx(const VectorXd &x, MatrixXd& C) {
	   VectorXd x_r, x_o;
	   parse_localizer_state(x, x_r, x_o);
	   C = MatrixXd::Zero(_NZ + _r->_NZ, _NX);
	   MatrixXd C_self, C_r;
	   Robot::dgdx(x, C_self);
	   _r->dgdx(x_r, C_r);
	   C.block(0,0,_NZ,_NX) = C_self;
	   C.block(_NZ,0,_r->_NZ,_r->_NX) = C_r;
   }

//   //collision linearization
   void dpdx(const VectorXd &x, MatrixXd& P) {
	   VectorXd x_r, x_o;
	   parse_localizer_state(x, x_r, x_o);
	   MatrixXd P_r;
	   _r->dpdx(x_r, P_r);
	   P = MatrixXd::Zero(P_r.rows(), _NX);
	   P.block(0,0,P_r.rows(),_r->_NX) = P_r;
   }

   // below is a biased version; biased in the sense that it uses _r->_NX samples
   // whereas the "correct" version uses _NX (where _NX = _r->_NX + 3*num_obj) samples
   // Caution: the taylor expansion *will* be incorrect if you use this

   //belief space collisions and linearizations
//   void bpenetration(const VectorXd& b, VectorXd& p) {
//	   VectorXd b_r;
//	   parse_localizer_belief_state(b, b_r);
//	   _r->bpenetration(b_r, p);
//   }

//   void dbpdx(const VectorXd &b, MatrixXd& P) {
//	   VectorXd b_r;
//	   parse_localizer_belief_state(b, b_r);
//	   MatrixXd P_r;
//	   _r->dbpdx(b_r, P_r);
//	   P = MatrixXd::Zero(P_r.rows(), _NB);
//	   P.block(0,0,P_r.rows(),_r->_NX) = P_r.block(0,0,P_r.rows(),_r->_NX);
//	   int ind_r = _r->_NX;
//	   int ind = _r->_NX + 3;
//	   for (int i = 0; i < _r->_NX; i++) {
//		   P.block(0,ind,P_r.rows(), _r->_NX - i ) = P_r.block(0,ind_r,P_r.rows(), _r->_NX -i);
//		   ind += _r->_NX-i ;
//		   ind_r += _r->_NX-i;
//		   ind += 3;
//	   }
//   }


   //Position linearization
   void dxyz(const VectorXd & x, MatrixXd& Jxyz) {
		VectorXd x_r, x_o;
		parse_localizer_state(x, x_r, x_o);
		MatrixXd Jxyz_r;
		_r->dxyz(x_r, Jxyz_r);
		Jxyz = MatrixXd::Zero(3, _NX);
		Jxyz.block(0, 0, 3, _r->_NX) = Jxyz_r;
   }

   void dquat(const VectorXd & x, MatrixXd& Jquat) {
		VectorXd x_r, x_o;
		parse_localizer_state(x, x_r, x_o);
		MatrixXd Jquat_r;
		_r->dxyz(x_r, Jquat_r);
		Jquat = MatrixXd::Zero(4, _NX);
		Jquat.block(0, 0, 4, _r->_NX) = Jquat_r;
   }

};



#endif
