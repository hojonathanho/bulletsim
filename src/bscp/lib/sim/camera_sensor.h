#ifndef _camera_sensor_h
#define _camera_sensor_h

#include "sensors.h"
#include <osg/Geode>
#include <osg/Drawable>
#include <osg/ShapeDrawable>
#include <osg/Math>
#include <Eigen/Geometry>

using namespace std; 
using namespace Eigen;

class CameraSensor : public Sensor
{
  public:
	int _num_obj;
    CameraSensor(int num_obj) : Sensor(num_obj*2) {
    	_num_obj = num_obj;
    }

    void observe(const VectorXd& x, VectorXd &z) {

    }

   osg::Node* draw(const MatrixXd& sensor_state, const Vector4d& color, osg::Group* parent) {

     cout << "CameraSensor::Draw is not implemented" << endl;

   }

};

#endif

