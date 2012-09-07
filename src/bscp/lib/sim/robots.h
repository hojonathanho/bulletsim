#ifndef _robots_h
#define _robots_h

#include <Eigen/Dense>
#include <iostream>
#include <osg/Drawable>
#include <osg/Group>
#include "sensors.h"
#include <vector>
#include <boost/function.hpp>


using namespace std; 
using namespace Eigen;


class Robot { 
  public:
    int _NX;
    int _NU; 
    double _eps;
    int _NZ;
    int _NB; 
    double _dt;
    typedef VectorXd (*SensorFunc)(Robot& r, const VectorXd&);
    typedef MatrixXd (*SensorFuncJacobian)(Robot& r, const VectorXd&);
    typedef VectorXd (*GoalFunc)(Robot& r, const VectorXd&);
    typedef MatrixXd (*GoalFuncJacboian)(Robot& r, const VectorXd&);



    vector<Sensor*> sensors;
    vector<SensorFunc> sensor_fns; 
    vector<SensorFuncJacobian> sensor_fns_jac;


   explicit Robot(const int NX, const int NU, const double eps, const double dt = 0.1)
    : _NX(NX)
    , _NU(NU)
    , _NZ(0)
    , _NB(_NX*(_NX+3)/2)
    , _eps(eps)
    , _dt(dt)
  {
    cout << "Initializing robot with parameters: " << endl; 
    cout << "NX = " << _NX << endl; 
    cout << "NU = " << _NU << endl;
    cout << "eps = " << _eps << endl;
    cout << "dt = " << _dt << endl;
    sensors.resize(0);
    sensor_fns.resize(0);

  }

   void greet();

   /* Must be implemented by every robot */ 
   virtual void dynamics(const VectorXd &x, const VectorXd &u, VectorXd &fxu) = 0; //f(x,u)
   virtual void penetration(const VectorXd& x, VectorXd& p) = 0; // p(x), note p(x) needs to be fixed dimension, i.e. p(x) \in R^P
   virtual double x_upper_limit(const int index) = 0;
   virtual double x_lower_limit(const int index) = 0;
   virtual double u_upper_limit(const int index) = 0;
   virtual double u_lower_limit(const int index) = 0;
   virtual void M(const VectorXd& x, MatrixXd& M) = 0; // sqrt(Q) = M 
   virtual void N(const VectorXd& x, MatrixXd& N) = 0; // sqrt(R) = N
   virtual osg::Node* draw(VectorXd x, Vector4d color, osg::Group* parent) =0;
   virtual osg::Node* draw_belief(VectorXd b, Vector4d mean_color, Vector4d ellipsoid_color, osg::Group* parent, double z_offset=0) =0; 
   virtual Vector3d xyz (const VectorXd &x) = 0;
   virtual Vector4d quat(const VectorXd &x) = 0;
   

   // Wrappers for sensors 
   void observe(const VectorXd& x, VectorXd& z);  // g(x)
   void attach_sensor(Sensor* sensor, SensorFunc g_i, SensorFuncJacobian dg_i = NULL);
   void draw_sensors(const VectorXd& x, const Vector4d& color, osg::Group *parent, double z_offset=0);
   void draw_sensor_trajectory(const vector<VectorXd>& X_bar, const Vector4d& color, osg::Group* parent, double z_offset=0);
   void draw_sensor_belief_trajectory(const vector<VectorXd>& B_bar, const Vector4d& color, osg::Group* parent, double z_offset=0);

   // Deterministic Belief Dynamics
   void belief_dynamics(const VectorXd& b, const VectorXd& u, VectorXd& bt1); 
   void belief_dynamics_fixed_model(const VectorXd& b, const VectorXd& u, const MatrixXd& A, const MatrixXd& C, VectorXd& bt1);
   void belief_noise(const VectorXd& b, const VectorXd& u, MatrixXd& W);

   // Utility
   // trajectory integration
   void forward_integrate(const VectorXd& x0, const vector<VectorXd>& u, const vector<VectorXd>& w, vector<VectorXd>& traj_x);
   // dynamics linearization
   virtual void dfdx(const VectorXd &x, const VectorXd &u, MatrixXd &A);
   virtual void dfdu(const VectorXd &x, const VectorXd &u, MatrixXd &B);
   virtual void df(const VectorXd &x, const VectorXd &u, MatrixXd &A, MatrixXd &B, VectorXd &c);
   virtual void df_trajectory(const vector<VectorXd>& X_bar, const vector<VectorXd>& U_bar, vector<MatrixXd>& As, vector<MatrixXd>& Bs, vector<VectorXd>& Cs);

   //observation linearization
   virtual void dgdx(const VectorXd &x, MatrixXd& C);
   virtual void dg(const VectorXd& x, MatrixXd& C, VectorXd& d);
   virtual void dtdx(const VectorXd &x, SensorFunc& f, MatrixXd& T);

   //goal linearization
   void dgoaldx(const VectorXd& x, GoalFunc& g, MatrixXd& Goal);
   void dgoal(const VectorXd& x, GoalFunc& g, GoalFuncJacboian& gj, MatrixXd& Goal, VectorXd& goal_offset);


   //collision linearization
   virtual void dpdx(const VectorXd &x, MatrixXd& P); //penetration linearization p(x)
   virtual void dp(const VectorXd &x, MatrixXd& P, VectorXd& p_offset);
   virtual void dp_trajectory(const vector<VectorXd>& X_bar, vector<MatrixXd>& Ps, vector<VectorXd>& p_offsets);
   //belief space collisions and linearizations
   virtual void bpenetration(const VectorXd& b, VectorXd& p);
   virtual void dbpdx(const VectorXd &b, MatrixXd& P);
   virtual void dbp(const VectorXd& b, MatrixXd& P, VectorXd& p_offset);
   virtual void dbp_trajectory(const vector<VectorXd>& B_bar, vector<MatrixXd>& Ps, vector<VectorXd>& p_offsets);

   //belief dynamics linearization
   virtual void dbdb(const VectorXd &b, const VectorXd &u, MatrixXd &A);
   virtual void dbdu(const VectorXd &b, const VectorXd &u, MatrixXd &B);
   virtual void db(const VectorXd &b, const VectorXd &u, MatrixXd &A, MatrixXd &B, VectorXd &c);
   virtual void db_trajectory(const vector<VectorXd>& B_bar, const vector<VectorXd>& U_bar, vector<MatrixXd>& As, vector<MatrixXd>& Bs, vector<VectorXd>& Cs);

   //Position utils linearization
   virtual void transform(const VectorXd& x, VectorXd& transform);
   virtual void dxyz (const VectorXd& x, MatrixXd& Jxyz );
   virtual void dquat(const VectorXd& x, MatrixXd& Jquat);
   virtual void dtransform(const VectorXd& x, MatrixXd& Jtransform);

   //Drawing
   vector<osg::Node*> draw_trajectory(vector<VectorXd> &traj_x, Vector4d color, osg::Group* parent);
   vector<osg::Node*> draw_belief_trajectory(vector<VectorXd> &traj_b, Vector4d mean_color, Vector4d ellispoid_color, osg::Group* parent, double z_offset=0); 

   //Transforms
   void unscented_transform_xyz(const VectorXd& mu_x, const MatrixXd& cov_x,
		   Vector3d& mu_y, Matrix3d& Sigma_y);
   void unscented_transform_goal(const VectorXd& mu_x, const MatrixXd& cov_x, const Robot::GoalFunc g,
		   VectorXd& mu_y, MatrixXd& Sigma_y);

};



#endif
