#ifndef _pr2_beacon_func_h
#define _pr2_beacon_func_h

#include "robots.h"

inline VectorXd PR2BeaconFunc(Robot& r, const VectorXd& x) {
	return r.xyz(x);
}

inline MatrixXd PR2BeaconFuncJacobian(Robot& r, const VectorXd& x) {
	MatrixXd ret;
	r.dxyz(x, ret);
	return ret;
}

#endif
