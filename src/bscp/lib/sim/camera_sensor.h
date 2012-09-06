#ifndef _camera_sensor_h
#define _camera_sensor_h

#include "sensors.h"
#include <osg/Geode>
#include <osg/Drawable>
#include <osg/ShapeDrawable>
#include <osg/Math>
#include <Eigen/Geometry>
#include "eigen_io_util.h"
#include <osgViewer/View>

using namespace std; 
using namespace Eigen;

inline void build_opengl_projection_from_intrinsics(Matrix4d &frustum,
		const double alpha, const double beta, const double skew, const double u0, const double v0,
   const int img_width, const int img_height, const double near, const double far ) {
	//http://tech.dir.groups.yahoo.com/group/OpenCV/message/38181
	frustum = Matrix4d::Zero();
	double fx = alpha;
	double fy = beta;
	double cx = u0;
	double cy = v0;

	frustum(0,0) = 2* fx/img_width;
	frustum(0,1) = 2*cx/img_width - 1;
	frustum(1,1) = 2*fy/img_height;
	frustum(1,2) = 2*cy/img_height - 1;
	frustum(2,2) = -(far+near)/(far-near);
	frustum(2,3) = -2*far*near/(far-near);
	frustum(3,2) = -1;

}

class CameraSensor : public Sensor
{
  public:
	int _num_obj;
	Matrix3d _KK;
	Matrix4d _frustum, _offset;
	double _far, _near;
	int _image_width, _image_height;
	osgViewer::View* _view;
	osg::Matrixd _view_projection;
	double _camera_draw_size;

    CameraSensor(int num_obj, Matrix3d &KK, int image_width, int image_height, Matrix4d fixed_offset = Matrix4d::Identity(), double camera_draw_size=0.005) : Sensor(num_obj*2) {
    	_num_obj = num_obj;
    	_KK = KK;
    	_far = 10000;
    	_near = 1;
    	_image_width = image_width;
    	_image_height = image_height;
    	_offset = fixed_offset;
    	_camera_draw_size = camera_draw_size;
    	_view = new osgViewer::View();


    	build_opengl_projection_from_intrinsics(_frustum,
    			-_KK(0,0), _KK(1,1), _KK(0,1), _KK(0,2),
    			_KK(1,2), _image_width, _image_height, _near, _far);

		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				_view_projection(j, i) = _frustum(i, j);
			}
		}

		_view->getCamera()->setProjectionMatrix(_view_projection);
	}

    inline Matrix4d ComputeExtrinsics(const Matrix4d& cam_world_transform) {
    	Matrix4d ret = Matrix4d::Identity();
    	ret.block(0,0,3,3) =  cam_world_transform.block(0,0,3,3).transpose();
    	ret.block(0,3,3,1) = - ret.block(0,0,3,3) * cam_world_transform.block(0,3,3,1);
    	return ret;
    }

    // x = [cam_x, cam_q, obj_1_x, obj_2_x,...]
    // all parameters are given in the world coordinate frame
    // i.e. cam_x is the position of the camera in the world coordinate frame
    void observe(const VectorXd& x, VectorXd &z) {
    	assert(x.rows() == 7 + _num_obj*3);
    	Vector3d cam_x = x.segment(0,3);

    	Quaterniond cam_q;
    	cam_q.coeffs() = x.segment(3,4);//Quaterniond(x(3), x(4), x(5), x(6));
    	//construct transform
    	Matrix4d C_w = Matrix4d::Identity();
    	C_w.block(0,0,3,3) = cam_q.toRotationMatrix();
    	C_w.block(0,3,3,1) = cam_x;
    	C_w  = C_w * _offset;
    	Matrix4d E = ComputeExtrinsics(C_w);

    	//project point
    	VectorXd obj_pos = x.segment(7, _num_obj*3);
    	z = VectorXd(_num_obj*2);
    	for (int i = 0; i < _num_obj; i++) {
    		Vector4d XXc = E * Vector4d(obj_pos(i*3), obj_pos(i*3+1), obj_pos(i*3+2), 1.0);
    		Vector3d pxy;
    		if (XXc(2) < 0) {
    		Vector3d Xn(XXc(0) / XXc(2), XXc(1) / XXc(2), 1.0);
				pxy = _KK * Xn;
				if (pxy(0) < 0)
					pxy(0) = 0;
				else if (pxy(0) > _image_width)
					pxy(0) = _image_width;

				if (pxy(1) < 0)
					pxy(1) = 0;
				else if (pxy(1) > _image_height)
					pxy(1) = _image_height;
    		} else {
    			pxy(0) = _image_width;
    			pxy(1) = _image_height;
    		}

    		double cx = _image_width/2;
    		double cy = _image_height/2;
    		pxy(0) = exp(- abs(pxy(0) - cx)/ _image_width );
    		pxy(1) = exp(- abs(pxy(1) - cy)/ _image_height );
    		//cout << pxy.transpose() << endl;
    		z.segment(i*2, 2) = pxy.segment(0,2);
    	}
    }
    //sensor state is the serialized transform (i.e. a vector)
    osg::Node* draw(const MatrixXd& sensor_state_vec, const Vector4d& color, osg::Group* parent, double z_offset=0) {
    	return drawSize(sensor_state_vec, color, parent, z_offset, _camera_draw_size);
    }


    //sensor state is the serialized transform (i.e. a vector)
   osg::Node* drawSize(const MatrixXd& sensor_state_vec, const Vector4d& color, osg::Group* parent, double z_offset=0, double size=0.005) {
	   osg::Matrixd model_view;
	   Matrix4d sensor_state;
	   vec2transform(sensor_state_vec, sensor_state);
	   Matrix4d offset_sensor_state = sensor_state * _offset;
	   for (int i = 0; i < 4; i++) {
		   for (int j = 0; j < 4; j++) {
			   model_view(j,i) = offset_sensor_state(i,j);
		   }
	   }
	   model_view(3,2) = offset_sensor_state(2,3) + z_offset;

	   _view->getCamera()->setViewMatrix(osg::Matrixd::inverse(model_view));
	   osg::Node* node = makeFrustumFromCamera(_view->getCamera(), size);
	   parent->addChild(node);

	   return node;
   }

   //sensor state is the serialized transform (i.e. a vector)
   // note that you still need to set the scene root for this function
	osgViewer::View* renderCameraView(const MatrixXd& sensor_state_vec, double z_offset=0) {
		Matrix4d sensor_state;
		vec2transform(sensor_state_vec, sensor_state);
		osgViewer::View* render_view = new osgViewer::View;
		render_view->setUpViewInWindow(0,0,_image_width,_image_height);
		render_view->getCamera()->setProjectionMatrix(_view_projection);
		osg::Matrixd model_view;
		Matrix4d offset_sensor_state = sensor_state * _offset;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				model_view(j, i) = offset_sensor_state(i, j);
			}
		}
		model_view(3, 2) = offset_sensor_state(2, 3) + z_offset;
		render_view->getCamera()->setViewMatrix(osg::Matrixd::inverse(model_view));

		return render_view;
	}

};

#endif

