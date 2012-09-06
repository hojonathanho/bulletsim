#ifndef _sensor_functions_h
#define _sensor_functions_h
#include "robots.h"
#include "localizer.h"
#include <boost/cast.hpp>

//inline VectorXd CarBeaconFunc(Robot& r, const VectorXd& x) {
//  return x.segment(0,2);
//}
//
//inline VectorXd LocalizerCarCameraFunc(Robot& r, const VectorXd& x) {
//	Localizer* l = static_cast<Localizer*>(&r);
//	VectorXd x_r, x_o;
//	l->parse_localizer_state(x, x_r, x_o);
//	VectorXd x_transform;
//	l->transform(x,x_transform);
//	VectorXd ret(7 + x_o.rows());
//	ret.segment(0,7) = x_transform;
//	ret.segment(7,x_o.rows()) = x_o;
//	return ret;
//}

//inline VectorXd LocalizerPR2CameraFunc(Robot& r, const VectorXd& x) {
//	Localizer* l = static_cast<Localizer*>(&r);
//	VectorXd x_r, x_o;
//	l->parse_localizer_state(x, x_r, x_o);
//	VectorXd x_transform;
//	l->transform(x,x_transform); // for right now
//	VectorXd ret(7 + x_o.rows());
//	ret.segment(0,7) = x_transform;
//	ret.segment(7,x_o.rows()) = x_o;
//	return ret;
//
//}

//inline VectorXd StateSensorFunc(Robot& r, const VectorXd& x) {
//  return x;
//}

//inline VectorXd PR2BeaconFunc(Robot& r, const VectorXd& x) {
//	return r.xyz(x);
//}
//
//inline MatrixXd PR2BeaconFuncJacobian(Robot& r, const VectorXd& x) {
//	MatrixXd ret;
//	r.dxyz(x, ret);
//	return ret;
//}

#endif

