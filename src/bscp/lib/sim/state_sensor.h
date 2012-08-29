#ifndef _state_sensor_h
#define _state_sensor_h

#include "sensors.h"
#include <osg/Geode>
#include <osg/Drawable>
#include <osg/ShapeDrawable>
#include <osg/Math>
#include <Eigen/Geometry>

using namespace std; 
using namespace Eigen;

class StateSensor : public Sensor 
{
  public:
    VectorXd _state_indices; 
    StateSensor(const VectorXd &state_indices) : Sensor(state_indices.rows()) {
      _state_indices = state_indices;
    }

    void observe(const VectorXd& x, VectorXd &z) {
      z = VectorXd(_NZ);
      for (int i = 0; i < _NZ; i++) { 
        z(i) = x(_state_indices(i)); 
      }
    }


   osg::Node* draw(const MatrixXd& sensor_state, const Vector4d& color, osg::Group* parent) {

     cout << "StateSensor::Draw is not implemented" << endl;
     return NULL; 

   }

};

#endif

