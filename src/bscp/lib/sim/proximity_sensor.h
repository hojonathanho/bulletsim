#ifndef _proximity_sensor_h
#define _proximity_sensor_h

#include "sensors.h"
#include "collision_util.h"
#include "simulation/basicobjects.h"
#include "osg_util.h"
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

class ProximitySensor : public Sensor
{
  public:
	double _radius;
	SphereObject::Ptr _sphere;
	Scene* _scene;
	ProximitySensor(double radius, Scene* scene) : Sensor(1) {
		_radius = radius;
		_scene = scene;
		btTransform initTransform = btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0));
		_sphere = SphereObject::Ptr(new SphereObject(0, _radius, initTransform));
	}

    void observe(const VectorXd& x, VectorXd &z) {

    	Vector3d sphere_center = x.segment(0,3);

    	btTransform pos = btTransform(btQuaternion(0,0,0,1), btVector3(x(0),x(1),x(2)));
    	_sphere->rigidBody->setCenterOfMassTransform(pos);
    	_scene->env->bullet->dynamicsWorld->addRigidBody(_sphere->rigidBody.get());
		vector<btVector3> points;
		vector<btVector3> normals;
		vector<double> dists;
    	calcCollisionInfo(_sphere->rigidBody.get(), _scene->bullet->dynamicsWorld, points, normals, dists);
    	double max_dist = 0;
    	for (int i = 0; i < dists.size(); i++) {
    		Vector3d point_sphere = toEigenVectord(points[i]);
    		//if (dists[i] < 0) {

    		//}
    		if (max_dist < abs(dists[i])) {
    			max_dist = abs(dists[i]);
    		}
    	}
    	//_scene->env->remove(_sphere);
    	_scene->env->bullet->dynamicsWorld->removeRigidBody(_sphere->rigidBody.get());

    	z = VectorXd(1);
    	z(0) = max(_radius - max_dist, 0.0);
    }


   osg::Node* draw(const MatrixXd& sensor_state, const Vector4d& color, osg::Group* parent, double z_offset=0) {

     Vector3d pos = sensor_state.col(0);
     Matrix3d cov = _radius * Matrix3d::Identity();
     osg::Node* sphere = drawEllipsoid(pos, cov, color);
     //parent->addChild(sphere);
     return sphere;

   }

};



#endif
