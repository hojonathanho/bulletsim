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
    vector<Sensor*> sensors;
    vector<SensorFunc> sensor_fns; 

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
   virtual double x_upper_limit(const int index) = 0;
   virtual double x_lower_limit(const int index) = 0;
   virtual double u_upper_limit(const int index) = 0;
   virtual double u_lower_limit(const int index) = 0;
   virtual void dpdx(const VectorXd &x, MatrixXd& P, VectorXd& p_offset) = 0; //penetration linearization p(x)
   virtual void M(const VectorXd& x, MatrixXd& M) = 0; // sqrt(Q) = M 
   virtual void N(const VectorXd& x, MatrixXd& N) = 0; // sqrt(R) = N
   virtual osg::Node* draw(VectorXd x, Vector4d color, osg::Group* parent) =0;
   virtual osg::Node* draw_belief(VectorXd b, Vector4d mean_color, Vector4d ellipsoid_color, osg::Group* parent, double z_offset=0) =0; 
   
   // Wrappers for sensors 
   void observe(const VectorXd& x, VectorXd& z); 
   void attach_sensor(Sensor* sensor, SensorFunc f); 

   // Deterministic Belief Dynamics
   void belief_dynamics(const VectorXd& b, const VectorXd& u, VectorXd& bt1); 

   // Utility
   // dynamics linearization
   void dfdx(const VectorXd &x, const VectorXd &u, MatrixXd &A); 
   void dfdu(const VectorXd &x, const VectorXd &u, MatrixXd &B); 
   void df(const VectorXd &x, const VectorXd &u, MatrixXd &A, MatrixXd &B, VectorXd &c);
   void df_trajectory(const vector<VectorXd>& X_bar, const vector<VectorXd>& U_bar, vector<MatrixXd>& As, vector<MatrixXd>& Bs, vector<VectorXd>& Cs);
   //observation linearization
   void dgdx(const VectorXd &x, MatrixXd& C); 
   void dg(const VectorXd& x, MatrixXd& C, VectorXd& d);
   //collision linearization
   void dp_trajectory(const vector<VectorXd>& X_bar, vector<MatrixXd>& Ps, vector<VectorXd>& p_offsets);
   void dbp_trajectory(const vector<VectorXd>& B_bar, vector<MatrixXd>& Ps, vector<VectorXd>& p_offsets);

   //belief dynamics linearization
   void dbdb(const VectorXd &b, const VectorXd &u, MatrixXd &A); 
   void dbdu(const VectorXd &b, const VectorXd &u, MatrixXd &B); 
   void db(const VectorXd &b, const VectorXd &u, MatrixXd &A, MatrixXd &B, VectorXd &c);
   void db_trajectory(const vector<VectorXd>& B_bar, const vector<VectorXd>& U_bar, vector<MatrixXd>& As, vector<MatrixXd>& Bs, vector<VectorXd>& Cs);

   //Drawing
   vector<osg::Node*> draw_trajectory(vector<VectorXd> &traj_x, Vector4d color, osg::Group* parent);

   vector<osg::Node*> draw_belief_trajectory(vector<VectorXd> &traj_b, Vector4d mean_color, Vector4d ellispoid_color, osg::Group* parent, double z_offset=0); 


};

#endif
