#ifndef _localizer_car_camera_func_h
#define _localizer_car_camera_func_h
#include "robots.h"
#include "localizer.h"
#include <boost/cast.hpp>

inline VectorXd LocalizerCarCameraFunc(Robot& r, const VectorXd& x) {
	Localizer* l = static_cast<Localizer*>(&r);
	VectorXd x_r, x_o;
	l->parse_localizer_state(x, x_r, x_o);
	VectorXd x_transform;
	l->transform(x,x_transform);
	VectorXd ret(7 + x_o.rows());
	ret.segment(0,7) = x_transform;
	ret.segment(7,x_o.rows()) = x_o;
	return ret;
}


#endif
