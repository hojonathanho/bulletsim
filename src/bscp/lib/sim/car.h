#ifndef _car_h
#define _car_h

#include "robots.h"
#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Drawable>
#include <osg/ShapeDrawable>
#include <osg/Math>
#include <Eigen/Geometry>
#include "eigen_io_util.h"
#include "osg_util.h"

using namespace std; 
using namespace Eigen;

class Car : public Robot 
{
  public:

    double _l;
    VectorXd _x;
    MatrixXd _M; bool M_set;
    MatrixXd _N; bool N_set; 

    Car(VectorXd &x) : Robot(4, 2, 1e-2, 0.1)
    {
      cout << "Initializing car" << endl;
      _l = 0.01;
      _x = x;
      M_set = false;
      N_set = false; 
    }

    void dynamics(const VectorXd &x, const VectorXd &u, VectorXd &fxu) {
      // implements x_{t+1} = f(x_t, u_t)
      
      double xp = x(0);
      double yp = x(1);
      double theta = x(2);
      double v = x(3);

      double a = u(0);
      double phi = u(1);

      double x_dot = v*cos(theta);
      double y_dot = v*sin(theta);
      double theta_dot = v * tan(phi) / _l;

      VectorXd xdot(4);
      xdot(0) = x_dot;
      xdot(1) = y_dot;
      xdot(2) = theta_dot;
      xdot(3) = a;

      fxu = x + _dt * xdot;

      /* Quick debug with a linear system */
      /*
      Matrix4d A; 
      A <<  1, 2, 3, 4,
            5, 6, 7, 8,
            9, 10, 11, 12,
            13, 14, 15, 16;
      A = A / 10; 
      MatrixXd B(4,2); 
      B << 8, 7, 6, 5, 4, 3, 2, 1;
      B = B / 2; 
      fxu = A*x + B*u;
      */
    }

    double x_upper_limit(const int index){ return  1e20; } //if you set this, beaware of belief space
    double x_lower_limit(const int index){ return -1e20; }
    double u_upper_limit(const int index){ return  1e20; }
    double u_lower_limit(const int index){ return -1e20; }

    void penetration(const VectorXd &x, VectorXd& p) {
    	p = VectorXd::Zero(0);
    }

//    void dpdx(const VectorXd &x, MatrixXd& P, VectorXd& p_offset){
//    	// no collisions right now, so:
//    	P = MatrixXd::Zero(0,_NX);
//    	p_offset = VectorXd::Zero(0);
//    }

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

    Vector3d xyz(const VectorXd& x) {
    	Vector3d ret(x(0), x(1), 0);
    	return ret;
    }


    osg::Node* draw(VectorXd x, Vector4d color, osg::Group* parent) {

      double xp = x(0);
      double yp = x(1);
      double theta = x(2);

      Matrix3d rot = Matrix3d::Identity();
      rot(0,0) = cos(theta);
      rot(0,1) = -sin(theta);
      rot(1,0) = sin(theta);
      rot(1,1) = cos(theta);

      Quaterniond q;
      q = rot;
      osg::Quat osg_q(q.x(), q.y(), q.z(), q.w());

      osg::Vec4 osg_color(color(0), color(1), color(2), color(3));
      osg::Box *car_shape = new osg::Box(osg::Vec3(xp, yp, 0.0), 0.01, 0.01, 0.02);
      car_shape->setRotation(osg_q);

      osg::ShapeDrawable *car = new osg::ShapeDrawable(car_shape);
      car->setColor(osg_color);
      osg::Geode *geode= new osg::Geode;
      geode->addDrawable(car);
      parent->addChild(geode);
      return geode; 
    }

    osg::Node* draw_belief(VectorXd b, Vector4d mean_color, Vector4d ellipsoid_color, osg::Group* parent, double z_offset=0) { 
      VectorXd x; MatrixXd rt_cov; 
      parse_belief_state(b, x, rt_cov);
      MatrixXd cov = rt_cov * rt_cov.transpose(); 

      Vector2d el_mean = x.segment(0,2);
      Matrix2d el_cov = cov.block(0,0,2,2); 

      osg::Group* g = new osg::Group;
      draw(x, mean_color, g); 
      g->addChild(drawEllipsoid2D(el_mean, z_offset, el_cov, ellipsoid_color));
      parent->addChild(g); 

      return g; 

    }

    void serialize(VectorXd &x, VectorXd &x_out) {
      x_out = x; 
    }

};


#endif
