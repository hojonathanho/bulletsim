#ifndef _car_beacon_func_h
#define _car_beacon_func_h
#include "robots.h"

inline VectorXd CarBeaconFunc(Robot& r, const VectorXd& x) {
  return x.segment(0,2);
}

#endif
