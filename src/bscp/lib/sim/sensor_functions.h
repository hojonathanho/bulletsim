#ifndef _sensor_functions_h
#define _sensor_functions_h

VectorXd CarBeaconFunc(const VectorXd& x) {
  return x.segment(0,2);
}

VectorXd StateSensorFunc(const VectorXd& x) {
  return x;  
}

#endif

