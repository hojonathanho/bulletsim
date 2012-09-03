#ifndef _sensor_functions_h
#define _sensor_functions_h
#include "robots.h"



inline VectorXd CarBeaconFunc(Robot& r, const VectorXd& x) {
  return x.segment(0,2);
}

inline VectorXd StateSensorFunc(Robot& r, const VectorXd& x) {
  return x;  
}

inline VectorXd PR2BeaconFunc(Robot& r, const VectorXd& x) {
	return r.xyz(x);
}

inline MatrixXd PR2BeaconFuncJacobian(Robot& r, const VectorXd& x) {
	MatrixXd ret;
	r.dxyz(x, ret);
	return ret;
}

#endif

