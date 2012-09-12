#ifndef _beacon_sensor_h
#define _beacon_sensor_h

#include "sensors.h"
#include <osg/Geode>
#include <osg/Drawable>
#include <osg/ShapeDrawable>
#include <osg/Math>
#include <Eigen/Geometry>

using namespace std; 
using namespace Eigen;

class BeaconSensor : public Sensor 
{
  public:
    VectorXd _pos;
    double _decay_coeff; 
    MatrixXd _N;
    bool N_set;
    BeaconSensor(const VectorXd& beacon_pos, const double decay_coeff=25) : Sensor(1) {
      _pos = beacon_pos;
      _decay_coeff = decay_coeff;
      N_set = false;
    }

    void observe(const VectorXd& x, VectorXd &z) {
      assert(x.rows() == _pos.rows());
      z = VectorXd(1);
      VectorXd pos_diff = x - _pos; 
      double denom = _decay_coeff*pos_diff.squaredNorm() + 1.0; 
      z(0) = 1.0 / (denom);
    }

    void rt_noise(const VectorXd& x, MatrixXd& N) {
    	assert(N_set == true);
    	N = _N;
    }

    void setN(const double n) {
    	_N = MatrixXd::Zero(1,1);
    	_N(0) = n;
    	N_set = true;
    }

   osg::Node* draw(const MatrixXd& sensor_state, const Vector4d& color, osg::Group* parent, double z_offset=0) {
     //VectorXd sensor_pos = sensor_state.col(0);
	 VectorXd sensor_pos = _pos;
     assert(sensor_pos.rows() == 2 || sensor_pos.rows()==3);
     double xp, yp, zp; 
     xp = sensor_pos(0);
     yp = sensor_pos(1);
     if (sensor_pos.rows() == 2) {
       zp = 0; 
     } else {
       zp = sensor_pos(2);
     }
      osg::Vec4 osg_color(color(0), color(1), color(2), color(3));
      osg::Cylinder *beacon_shape = new osg::Cylinder(osg::Vec3(xp, yp, zp+0.125), 0.01,0.25);

      osg::ShapeDrawable *beacon = new osg::ShapeDrawable(beacon_shape);
      beacon->setColor(osg_color);
      osg::Geode *geode = new osg::Geode; 
      geode->addDrawable(beacon);
      parent->addChild(geode);
      return geode; 


   }

};

#endif

