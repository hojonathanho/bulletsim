#ifndef _sensors_h
#define _sensors_h

#include <Eigen/Dense>
#include <iostream>
#include <osg/Node>
#include <osg/Group>

using namespace std; 
using namespace Eigen;

typedef VectorXd (*SensorFunc)(const VectorXd&);


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
   // x = sensor_state(robot_state); ,  z = g(x) 
   virtual void observe(const VectorXd &x, VectorXd &z) = 0;
   
   virtual osg::Node* draw(const MatrixXd& sensor_state, const Vector4d& color, osg::Group *parent) =0;

};


VectorXd CarBeaconFunc(const VectorXd& x);


#endif
