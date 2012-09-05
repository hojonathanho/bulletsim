#ifndef _sensors_h
#define _sensors_h

#include <Eigen/Dense>
#include <iostream>
#include <osg/Node>
#include <osg/Group>

using namespace std; 
using namespace Eigen;


class Sensor { 
  public:
    int _NZ;

   explicit Sensor(const int NZ)
    : _NZ(NZ)
  {
    cout << "Initializing sensor with parameters: " << endl; 
    cout << "NZ = " << _NZ << endl;
  }
   
   int NZ() { 
     return _NZ;
   }
   // x = sensor_state(robot_state); , z = g(x)
   virtual void observe(const VectorXd &x, VectorXd &z) = 0;
   
   // x = sensor_state(robot_state); C = \partial g
   // this is for an use in a chain rule; implementing classes may (should) override with an analytical version
   virtual void dgdx(const VectorXd& x, double eps, MatrixXd& C) {
	   int NX = x.rows();
	   C = MatrixXd::Zero(_NZ, NX);
	   for (int i = 0; i < NX; i++) {
	     VectorXd eps_vec = VectorXd::Zero(NX);
	     eps_vec(i) = eps;
	     VectorXd x_pos = x + eps_vec;
	     VectorXd x_neg = x - eps_vec;
	     VectorXd gx_pos(NX); VectorXd gx_neg(NX);
	     observe(x_pos, gx_pos);
	     observe(x_neg, gx_neg);
	     C.col(i) = (gx_pos - gx_neg) / (2*eps);
	   }
   }

   virtual osg::Node* draw(const MatrixXd& sensor_state, const Vector4d& color, osg::Group *parent, double z_offset=0) =0;

};


#endif
