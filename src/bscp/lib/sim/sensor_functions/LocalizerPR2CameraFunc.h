#ifndef _localizer_pr2_camera_func_h
#define _localizer_pr2_camera_func_h
#include "robots.h"
#include "localizer.h"
#include "bscp_pr2.h"
#include <boost/cast.hpp>

inline VectorXd LocalizerPR2AttachedCameraFunc(Robot& r, const VectorXd& x) {
	Localizer* l = static_cast<Localizer*>(&r);
	VectorXd x_r, x_o;
	l->parse_localizer_state(x, x_r, x_o);
	PR2_SCP* pr2_scp = static_cast<PR2_SCP*>(l->_r);
	VectorXd x_transform = pr2_scp->camera_transform(x_r);
	VectorXd ret(7 + x_o.rows());
	ret.segment(0,7) = x_transform;
	ret.segment(7,x_o.rows()) = x_o;
	return ret;
}


//FINISH ME
inline MatrixXd LocalizerPR2AttachedCameraFuncJac(Robot& r, const VectorXd& x) {
	Localizer* l = static_cast<Localizer*>(&r);
	VectorXd x_r, x_o;
	l->parse_localizer_state(x, x_r, x_o);
	PR2_SCP* pr2_scp = static_cast<PR2_SCP*>(l->_r);
	MatrixXd ret = MatrixXd::Zero(7 + x_o.rows(), l->_NX);
	MatrixXd Jcam_transform;
	pr2_scp->dcamera_transform(x_r, Jcam_transform);
	ret.block(0,0,7,pr2_scp->_NX) = Jcam_transform;
	ret.block(7,7,x_o.rows(), x_o.rows()) = MatrixXd::Identity(x_o.rows(), x_o.rows());
	return ret;

}



#endif
