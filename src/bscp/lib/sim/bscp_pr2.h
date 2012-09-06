#ifndef _bscp_pr2_h
#define _bscp_pr2_h

#include "robots.h"
#include "robots/pr2.h"
#include "boost/foreach.hpp"
#include "boost/multi_array.hpp"
#include "simulation/util.h"
#include "utils/conversions.h"
#include "utils/clock.h"
#include "simulation/fake_gripper.h"
#include "osg_util.h"
#include "eigen_io_util.h"

using namespace std; 
using namespace Eigen;

struct Collision {
	const btCollisionObject* m_obj0;
	const btCollisionObject* m_obj1;
	btVector3 m_world0;
	btVector3 m_world1;
	btVector3 m_normal;
	btScalar m_distance;
	Collision(const btCollisionObject* obj0, const btCollisionObject* obj1, const btVector3& world0, const btVector3& world1, const btVector3& normal, btScalar distance) :
		m_obj0(obj0), m_obj1(obj1), m_world0(world0), m_world1(world1), m_normal(normal), m_distance(distance) {
	}
	Collision(const Collision& c) :
		m_obj0(c.m_obj0), m_obj1(c.m_obj1), m_world0(c.m_world0), m_world1(c.m_world1), m_normal(c.m_normal), m_distance(c.m_distance) {
	}
};

struct CollisionCollector: public btCollisionWorld::ContactResultCallback {
	std::vector<Collision> m_collisions;
	btScalar addSingleResult(btManifoldPoint& pt, const btCollisionObject *colObj0, int, int, const btCollisionObject *colObj1, int, int) {
		m_collisions.push_back(Collision(colObj0, colObj1, pt.m_positionWorldOnA, pt.m_positionWorldOnB, pt.m_normalWorldOnB, pt.m_distance1));
		return 0;
	}
};

class BulletRaveSyncer {
public:
	std::vector<OpenRAVE::KinBody::LinkPtr> m_links; // what links are part of the object we're planning with
	std::vector<btRigidBody*> m_bodies; // what rigid bodies are part of the object we're planning with
	BulletRaveSyncer(const std::vector<OpenRAVE::KinBody::LinkPtr>& links, const std::vector<btRigidBody*>& bodies) :
		m_links(links), m_bodies(bodies) {}
	void updateBullet() {
		 for (int iBody = 0; iBody < m_bodies.size(); ++iBody) {
		    m_bodies[iBody]->setCenterOfMassTransform(util::toBtTransform(m_links[iBody]->GetTransform(), GeneralConfig::scale));
		  }
	}
};


class PR2_SCP : public Robot 
{
  public:

    VectorXd _x;
    RaveRobotObject::Ptr _pr2;
    RaveRobotObject::Manipulator::Ptr _manip;
    vector<int> _active_dof_indices;
    vector<double> _upper_joint_limits;
    vector<double> _lower_joint_limits; 
    vector<KinBody::LinkPtr> _arm_links;
    vector<KinBody::JointPtr> _arm_joints;
    vector<btRigidBody*> _bt_arm_links;
    vector<int> _chain_depth_of_bodies;
    btCollisionWorld* _world;
    BulletRaveSyncer* _sync;


    MatrixXd _M; bool M_set;
    MatrixXd _N; bool N_set; 

    PR2_SCP(RaveRobotObject::Ptr pr2, vector<int> active_dof_indices, btCollisionWorld *world, RaveRobotObject::Manipulator::Ptr manip) : Robot(active_dof_indices.size(),active_dof_indices.size(),1e-2,0.1)   {

      cout << "Initializing PR2_SCP" << endl;
      _pr2 = pr2;
      _active_dof_indices = active_dof_indices;
      _manip = manip;
      _world = world;
      _pr2->robot->GetDOFLimits(_lower_joint_limits, _upper_joint_limits, _active_dof_indices); 
      cout << "Robot joint limits" << endl;
      for (int i = 0 ; i < _active_dof_indices.size(); i++) {
        cout << "Link " << active_dof_indices[i] << ": "
          << _lower_joint_limits[i] << ", " << _upper_joint_limits[i] << endl; 
      }

      _arm_links.resize(0);
      _arm_joints.resize(0);
      getArmKinInfo(_pr2->robot,_arm_links, _arm_joints);
      for (int i = 0; i < _arm_links.size(); i++) {
      	  _bt_arm_links.push_back( _pr2->associatedObj(_arm_links[i])->rigidBody.get() );
      }
      _sync = new BulletRaveSyncer(_arm_links, _bt_arm_links);


      M_set = false;
      N_set = false;

    }

    double x_upper_limit(const int i) {
    	if (i < _NX) {
    		return _upper_joint_limits[i];
    	} else {
    		return 1e20;
    	}
    }
    double x_lower_limit(const int i) {
    	if (i < _NX) {
    		return _lower_joint_limits[i];
    	} else {
    		return -1e20;
    	}
    }
    double u_upper_limit(const int i) { return   0.1; }
    double u_lower_limit(const int i) { return  -0.1; }

    vector<btRigidBody*> get_bullet_links() { return _bt_arm_links; }
    vector<KinBody::LinkPtr> get_OpenRAVE_links() { return _arm_links; }

    void penetration(const VectorXd& x_bar, VectorXd &p) {
    	//vector<double> currentDOF = _pr2->getDOFValues(_active_dof_indices);
    	_pr2->robot->SetDOFValues(toVec(x_bar), 1, _active_dof_indices); // does NOT update Bullet
    	_sync->updateBullet(); // explicitly update bullet

    	//_pr2->setDOFValues(_active_dof_indices, toVec(x_bar)); //updates Bullet
    	 p = VectorXd::Zero(_arm_links.size());
		vector<int> num_collisions(_arm_links.size());
		for (int i = 0; i < _arm_links.size(); i++) {
			btRigidBody* bt_link = _bt_arm_links[i];
			vector<btVector3> points;
			vector<btVector3> normals;
			vector<double> dists;
			calcCollisionInfo(bt_link, _world, points, normals, dists);
			vector<double> jac_v;

			for (int j = 0; j < points.size(); j++) {
				num_collisions[i] += 1;
				p(i) += dists[j];
			}
		}
		for (int i = 0; i < _arm_links.size(); i++) {
			if (num_collisions[i] > 0) {
				p(i) /= num_collisions[i];
			}
		}
        //_pr2->setDOFValues(_active_dof_indices, currentDOF);
    }

    void dfdx(const VectorXd &x, const VectorXd &u, MatrixXd &A) {
    	A = MatrixXd::Identity(_NX, _NX);
    }
    void dfdu(const VectorXd &x, const VectorXd &u, MatrixXd &B) {
    	B = MatrixXd::Identity(_NX, _NX);
    }

    void dbdu(const VectorXd &b, const VectorXd &u, MatrixXd &B) {
    	B = MatrixXd::Zero(_NB, _NX);
    	B.block(0,0,_NX,_NX) = MatrixXd::Identity(_NX, _NX);
    }



    void dpdx(const VectorXd& x, MatrixXd& D) {
    	VectorXd d_offset;
    	dp_analytical(x, D, d_offset);
    }


	void dp(const VectorXd& x_bar, MatrixXd& D, VectorXd& d_offset) {
		dp_analytical(x_bar, D, d_offset);
	}

    void dp_analytical(const VectorXd& x_bar, MatrixXd& D, VectorXd& d_bar) {
    	//returns d(x) \approx Dx + d_bar; d_bar = d(x_bar) - Dx_bar;
    	//vector<double> currentDOF = _pr2->getDOFValues(_active_dof_indices);
    	_pr2->setDOFValues(_active_dof_indices, toVec(x_bar)); //updates Bullet

    	/*double max_dist = 0.0;
    	btVector3 max_normal, max_point;
    	int max_link_ind;
    	bool exists_collision = false;
    	for (int i = 0; i < _arm_links.size(); i++) {
    		 btRigidBody* bt_link = _bt_arm_links[i];
    		 vector<btVector3> points;
    		 vector<btVector3> normals;
    		 vector<double> dists;
    		 calcCollisionInfo(bt_link, _world, points, normals, dists);
    		 for (int j = 0; j < points.size(); j++) {
    			 //cout << "dists j = " << dists[j] << endl;
    			 if (dists[j] < max_dist) {
        			 exists_collision = true;
    				 max_dist = dists[j];
    				 max_point = points[j];
    				 max_normal = normals[j];
    				 max_link_ind = i;
    			 }
    			 if (-dists[j] > max_dist) {
    				 max_dist = -dists[j];
    				 max_point = points[j];
    				 max_normal = -normals[j];
    				 max_link_ind = i;
    			 }
    		 }
    	}

    	D = MatrixXd::Zero(1,x_bar.rows());
    	d_bar = VectorXd::Zero(1);

    	if (exists_collision == true) {
    		vector<double> jac_v(x_bar.rows()*3,0);
    		RaveVector<double> p = util::toRaveVector(max_point);
    		_pr2->robot->CalculateActiveJacobian(_arm_links[max_link_ind]->GetIndex(), p, jac_v);
    		MatrixXd jac(3, x_bar.rows());
    		for (int r = 0; r < 3; r++) {
    			for (int c = 0; c < x_bar.rows(); c++) {
    				jac(r,c) = jac_v[r*x_bar.rows() + c];
    			}
    		}
    		VectorXd d = jac.transpose()*toEigenVectord(max_normal);
    		D.row(0) = d.transpose();
    		d_bar(0) = max_dist - D.row(0)*x_bar;
    	}*/


    	D = MatrixXd::Zero(_arm_links.size(),x_bar.rows());
    	d_bar = VectorXd::Zero(_arm_links.size());
    	vector<int> num_collisions(_arm_links.size());
		for (int i = 0; i < _arm_links.size(); i++) {
			btRigidBody* bt_link = _bt_arm_links[i];
			vector<btVector3> points;
			vector<btVector3> normals;
			vector<double> dists;
			calcCollisionInfo(bt_link, _world, points, normals, dists);
			vector<double> jac_v;

			for (int j = 0; j < points.size(); j++) {
				num_collisions[i] += 1;
				RaveVector<double> p = util::toRaveVector(points[j]);
				_pr2->robot->CalculateActiveJacobian(_arm_links[i]->GetIndex(),
						p, jac_v);
				MatrixXd jac(3, x_bar.rows());
				for (int r = 0; r < 3; r++) {
					for (int c = 0; c < x_bar.rows(); c++) {
						jac(r, c) = jac_v[r * x_bar.rows() + c];
					}
				}
				VectorXd d = jac.transpose() * toEigenVectord(normals[j]);
				D.row(i) += d.transpose();
				d_bar(i) += dists[j] - d.transpose()*x_bar;
			}
    	}
		for (int i = 0; i < _arm_links.size(); i++) {
			if (num_collisions[i] > 0) {
				D.row(i) /= num_collisions[i];
				d_bar(i) /= num_collisions[i];
			}
		}

    	/*
    	vector<VectorXd> gradients(0);
    	vector<double> offsets(0);


        for (int i = 0; i < _arm_links.size(); i++) {
      	  //btRigidBody* bt_link = _pr2->associatedObj(_arm_links[i])->rigidBody.get(); // can precache this
      	  btRigidBody* bt_link = _bt_arm_links[i];
          vector<btVector3> points;
      	  vector<btVector3> normals;
      	  vector<double> dists;
      	  calcCollisionInfo(bt_link, _world, points, normals, dists);
      	  vector<double> jac_v;

      	  for (int j = 0; j < points.size(); j++) {

      		  RaveVector<double> p = util::toRaveVector(points[j]);
      	  	  //_pr2->robot->CalculateActiveJacobian(_manip->manip->GetEndEffector()->GetIndex(), p, jac_v);
      	  	  _pr2->robot->CalculateActiveJacobian(_arm_links[i]->GetIndex(), p, jac_v);
      		  MatrixXd jac(3,x_bar.rows());
      	  	  for (int r = 0; r < 3; r++) {
      	  		  for (int c = 0; c < x_bar.rows(); c++) {
      	  			  jac(r,c) = jac_v[r*x_bar.rows() + c];
      	  		  }
      	  	  }
      	  	  VectorXd d = jac.transpose()*toEigenVectord(normals[j]);
      	  	  gradients.push_back(d);
      	  	  offsets.push_back(dists[j] - d.transpose()*x_bar);
      	  }
        }
        D = MatrixXd(gradients.size(), x_bar.rows());
        d_bar = VectorXd(offsets.size());
        for (int i = 0 ; i < gradients.size(); i++) {
        	D.row(i) = gradients[i];
        	d_bar(i) = offsets[i];
        }
        */

        //_pr2->setDOFValues(_active_dof_indices, currentDOF);
    }

    void getArmKinInfo(const RobotBasePtr& robot, std::vector<KinBody::LinkPtr>& armLinks, std::vector<KinBody::JointPtr>& armJoints) {
      int rootLinkInd               = robot->GetLink("torso_lift_link")->GetIndex();
      cout << "rootLinkInd = " << rootLinkInd << endl;
      BOOST_FOREACH(int ind, _active_dof_indices) armJoints.push_back(robot->GetJointFromDOFIndex(ind));
      KinBody::JointPtr& firstJoint = armJoints[0];

      BOOST_FOREACH(KinBody::LinkPtr link, robot->GetLinks()) {
        vector<KinBody::JointPtr> jointChain;
        robot->GetChain(rootLinkInd, link->GetIndex(), jointChain);
        if (link->GetGeometries().size() && count(jointChain.begin(), jointChain.end(), firstJoint)) armLinks.push_back(link);
      }

      int nLinks                    = armLinks.size();

    }

    void calcCollisionInfo(btRigidBody* body, btCollisionWorld* world, vector<btVector3>& points, vector<btVector3>& normals, vector<double>& dists) {
      CollisionCollector collisionCollector;
      world->contactTest(body, collisionCollector);
      int nColl = collisionCollector.m_collisions.size();
      points.resize(nColl);
      normals.resize(nColl);
      dists.resize(nColl);

      for (int iColl = 0; iColl < nColl; iColl++) {
        Collision &collision = collisionCollector.m_collisions[iColl];
        points[iColl] = (collision.m_obj0 == body) ? collision.m_world0 : collision.m_world1;
        normals[iColl] = (collision.m_obj0 == body) ? collision.m_normal : -collision.m_normal;
        dists[iColl] = collision.m_distance;
      }
    }


    void dynamics(const VectorXd &x, const VectorXd &u, VectorXd &fxu) {
      fxu = x + u;
      for (int i = 0; i < _NX; i++) {
        if (fxu[i] < _lower_joint_limits[i]) fxu[i] = _lower_joint_limits[i];
        if (fxu[i] > _upper_joint_limits[i]) fxu[i] = _upper_joint_limits[i];
      }
    }


   void M(const VectorXd& x, MatrixXd& M) {
     assert(M_set == true);
     M = _M; 
   }

   void N(const VectorXd& x, MatrixXd& N) {
     assert(N_set == true);
     N = _N; 
   }

   void set_M(const MatrixXd& M) {
     _M = M;
     M_set = true; 
   }

   void set_N(const MatrixXd& N) {
     _N = N;
     N_set = true;
   }

    osg::Node* draw(VectorXd x, Vector4d color, osg::Group* parent) {
      return NULL; 
    }

    osg::Node* draw_belief(VectorXd b, Vector4d mean_color, Vector4d ellipsoid_color, osg::Group* parent, double z_offset=0) { 
      return NULL;
    }

    Vector3d camera_xyz(const VectorXd& x) {
		_pr2->robot->SetDOFValues(toVec(x), 1, _active_dof_indices);
		btTransform link_transform = util::toBtTransform(
				_arm_links[7]->GetTransform(), GeneralConfig::scale);
		Affine3f eig_transform = toEigenTransform(link_transform);
		Vector3f xyz = eig_transform.translation();
		Vector3d ret;
 		ret(0) = xyz(0);
		ret(1) = xyz(1);
		ret(2) = xyz(2);
		return ret;
    }

    void dcamera_xyz(const VectorXd& x, MatrixXd& Jxyz) {
       	//vector<double> currentDOF = _pr2->getDOFValues(_active_dof_indices);
        _pr2->robot->SetDOFValues(toVec(x), 1, _active_dof_indices);
    	vector<double> jac_v;
    	Vector3d current_point = camera_xyz(x);
		RaveVector<double> p = util::toRaveVector(btVector3(current_point(0), current_point(1), current_point(2)));
		_pr2->robot->CalculateActiveJacobian(_arm_links[7]->GetIndex(),
				p, jac_v);
		Jxyz = MatrixXd::Zero(3, x.rows());
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < x.rows(); c++) {
				Jxyz(r, c) = jac_v[r * x.rows() + c];
			}
		}
       // _pr2->robot->SetDOFValues(currentDOF, 1, _active_dof_indices);
    }

    Vector4d camera_quat(const VectorXd &x) {
		_pr2->robot->SetDOFValues(toVec(x), 1, _active_dof_indices);
		btTransform link_transform = util::toBtTransform(
				_arm_links[7]->GetTransform(), GeneralConfig::scale);
		Affine3f eig_transform = toEigenTransform(link_transform);
        Quaternionf quat = Quaternionf(eig_transform.rotation());
        Vector4f quat_vec = quat.coeffs(); // x y z w
        Vector4d ret;
        ret(0) = quat_vec(0);
        ret(1) = quat_vec(1);
        ret(2) = quat_vec(2);
        ret(3) = quat_vec(3);
    }

    void dcamera_quat(const VectorXd& x, MatrixXd& Jquat) {
    	//http://openrave.org/docs/latest_stable/geometric_conventions/
    	// note openrave quaternions are w x y z
    	// while eigen/bullet quaternions are x y z w

       	//vector<double> currentDOF = _pr2->getDOFValues(_active_dof_indices);
        _pr2->robot->SetDOFValues(toVec(x), 1, _active_dof_indices);
    	vector<double> jac_v;
    	Vector4d current_q = camera_quat(x);
    	RaveVector<double> q = RaveVector<double>(current_q(3), current_q(0), current_q(1), current_q(2));
		_pr2->robot->CalculateActiveRotationJacobian(_arm_links[7]->GetIndex(),
				q, jac_v);
		MatrixXd Jquat_wrong_order = MatrixXd::Zero(4, x.rows());
		for (int r = 0; r < 4; r++) {
			for (int c = 0; c < x.rows(); c++) {
				Jquat_wrong_order(r, c) = jac_v[r * x.rows() + c];
			}
		}
		Jquat = MatrixXd::Zero(4, x.rows());
		Jquat.block(0,0,3,x.rows()) = Jquat_wrong_order.block(1,0,3,x.rows());
		Jquat.block(3,0,1,x.rows()) = Jquat_wrong_order.block(0,0,1,x.rows());

       // _pr2->robot->SetDOFValues(currentDOF, 1, _active_dof_indices);
    }

    VectorXd camera_transform(const VectorXd& x) {
    	Vector3d cam_xyz = camera_xyz(x);
    	Vector4d cam_quat = camera_quat(x);
    	VectorXd ret(7);
    	ret.segment(0,3) = cam_xyz;
    	ret.segment(3,4) = cam_quat;

//        _pr2->robot->SetDOFValues(toVec(x), 1, _active_dof_indices);
//        btTransform link_transform = util::toBtTransform(_arm_links[6]->GetTransform(), GeneralConfig::scale);
//        Affine3f eig_transform = toEigenTransform(link_transform);
//        Vector3f xyz = eig_transform.translation();
//        Quaternionf quat = Quaternionf(eig_transform.rotation());
//        Vector4f quat_vec = quat.coeffs(); // x y z w
//        VectorXd ret(7);
//        ret(0) = xyz(0);
//        ret(1) = xyz(1);
//        ret(2) = xyz(2);
//        ret(3) = quat_vec(0);
//        ret(4) = quat_vec(1);
//        ret(5) = quat_vec(2);
//        ret(6) = quat_vec(3);
        return ret;
    }

    void dcamera_transform(const VectorXd& x, MatrixXd& Jtransform) {
    	MatrixXd Jxyz, Jquat;
    	dxyz(x, Jxyz);
    	dquat(x, Jquat);

    	Jtransform = MatrixXd(7, _NX);
    	Jtransform.block(0,0,3,_NX) = Jxyz;
    	Jtransform.block(3,0,4,_NX) = Jquat;
    }

    Vector3d xyz(const VectorXd& x) {
       	//vector<double> currentDOF = _pr2->getDOFValues(_active_dof_indices);
        _pr2->robot->SetDOFValues(toVec(x), 1, _active_dof_indices);
        btTransform manip_transform = _manip->getTransform();
        Affine3f eig_transform = toEigenTransform(manip_transform);
        Vector3f ret_f = eig_transform.translation();
        Vector3d ret = Vector3d(ret_f(0), ret_f(1), ret_f(2));
        //_pr2->robot->SetDOFValues(currentDOF, 1, _active_dof_indices);
        return ret;
    }

    Vector4d quat(const VectorXd& x) {

       	//vector<double> currentDOF = _pr2->getDOFValues(_active_dof_indices);
        _pr2->robot->SetDOFValues(toVec(x), 1, _active_dof_indices);
        btTransform manip_transform = _manip->getTransform();
        Affine3f eig_transform = toEigenTransform(manip_transform);
        Quaternionf quat = Quaternionf(eig_transform.rotation());
        Vector4f ret_f = quat.coeffs(); // x y z w
        Vector4d ret = Vector4d(ret_f(0), ret_f(1), ret_f(2), ret_f(3));
        //_pr2->robot->SetDOFValues(currentDOF, 1, _active_dof_indices);
        return ret;
    }

    void dxyz(const VectorXd& x, MatrixXd& Jxyz) {

       	//vector<double> currentDOF = _pr2->getDOFValues(_active_dof_indices);
        _pr2->robot->SetDOFValues(toVec(x), 1, _active_dof_indices);
    	vector<double> jac_v;
    	Vector3d current_point = xyz(x);
		RaveVector<double> p = util::toRaveVector(btVector3(current_point(0), current_point(1), current_point(2)));
		_pr2->robot->CalculateActiveJacobian(_manip->manip->GetEndEffector()->GetIndex(),
				p, jac_v);
		Jxyz = MatrixXd::Zero(3, x.rows());
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < x.rows(); c++) {
				Jxyz(r, c) = jac_v[r * x.rows() + c];
			}
		}


       // _pr2->robot->SetDOFValues(currentDOF, 1, _active_dof_indices);
    }


    void dquat(const VectorXd& x, MatrixXd& Jquat) {
    	//http://openrave.org/docs/latest_stable/geometric_conventions/
    	// note openrave quaternions are w x y z
    	// while eigen/bullet quaternions are x y z w

       	//vector<double> currentDOF = _pr2->getDOFValues(_active_dof_indices);
        _pr2->robot->SetDOFValues(toVec(x), 1, _active_dof_indices);
    	vector<double> jac_v;
    	Vector4d current_q = quat(x);
    	RaveVector<double> q = RaveVector<double>(current_q(3), current_q(0), current_q(1), current_q(2));
		_pr2->robot->CalculateActiveRotationJacobian(_manip->manip->GetEndEffector()->GetIndex(),
				q, jac_v);
		MatrixXd Jquat_wrong_order = MatrixXd::Zero(4, x.rows());
		for (int r = 0; r < 4; r++) {
			for (int c = 0; c < x.rows(); c++) {
				Jquat_wrong_order(r, c) = jac_v[r * x.rows() + c];
			}
		}
		Jquat = MatrixXd::Zero(4, x.rows());
		Jquat.block(0,0,3,x.rows()) = Jquat_wrong_order.block(1,0,3,x.rows());
		Jquat.block(3,0,1,x.rows()) = Jquat_wrong_order.block(0,0,1,x.rows());

       // _pr2->robot->SetDOFValues(currentDOF, 1, _active_dof_indices);
    }
};

class PR2_SCP_Plotter {
public:
  PR2_SCP* m_pr2;
  std::vector<BulletObject::Ptr> m_origs;
  vector<vector<FakeObjectCopy::Ptr> > m_fakes;
  //BasicArray<FakeObjectCopy::Ptr> m_fakes;
  Scene* m_scene;
  osg::Group* m_osgRoot;
  PR2_SCP_Plotter(PR2_SCP* pr2, Scene* scene, int length) {
	  m_pr2 = pr2;
	  m_scene = scene;
	  m_origs.resize(0);
	  BOOST_FOREACH(KinBody::LinkPtr link, m_pr2->get_OpenRAVE_links())
	  	  m_origs.push_back(m_pr2->_pr2->associatedObj(link));
	  m_osgRoot = scene->env->osg->root.get();
	  m_fakes.resize(length);
	  for (int j = 0; j < length; j++) m_fakes[j].resize(m_origs.size());
	  for (int iPlot = 0; iPlot < length; ++iPlot) {
	      for (int iObj = 0; iObj < m_origs.size(); ++iObj) {
	        FakeObjectCopy::Ptr& fake = m_fakes[iPlot][iObj];
	        fake.reset(new FakeObjectCopy(m_origs[iObj]));
	        fake->makeChildOf(m_osgRoot);
	      }
	  }

  }
  void draw_trajectory(vector<VectorXd>& traj, const Vector4d& color) {
	  vector<double> curDOFVals = m_pr2->_pr2->getDOFValues(m_pr2->_active_dof_indices);
	   for (int iPlot = 0; iPlot < m_fakes.size(); ++iPlot) {
	    	m_pr2->_pr2->setDOFValues(m_pr2->_active_dof_indices, toVec(traj[iPlot])); //updates Bullet
	    	for (int iObj = 0; iObj < m_origs.size(); ++iObj) {
	    		m_fakes[iPlot][iObj]->setTransform(m_origs[iObj]->rigidBody->getCenterOfMassTransform());
	    		m_fakes[iPlot][iObj]->setColor(toOSGVector(color));
	    	}
	   }
	   m_pr2->_pr2->setDOFValues(m_pr2->_active_dof_indices, curDOFVals); // updates Bullet

	   //m_scene->step(0); //this seems to be evil?
  }

  void draw_belief_trajectory(vector<VectorXd> &traj, const Vector4d& mean_color,
		  const Vector4d& cov_color, osg::Group* parent) {

	  int T = traj.size();
	  vector<VectorXd> mean(T);
	  vector<MatrixXd> rt_cov(T);
	  for (int t = 0; t < T; t++) {
		  parse_belief_state(traj[t], mean[t], rt_cov[t]);
	  }
	  draw_trajectory(mean, mean_color);

	  for (int t = 0; t < T; t++) {
		  Vector3d pos_x;
		  Matrix3d cov_x;
		  m_pr2->unscented_transform_xyz(mean[t], rt_cov[t]*rt_cov[t].transpose(),
				  pos_x, cov_x);
		  parent->addChild(drawEllipsoid(pos_x,cov_x,cov_color));
	  }
  }


};




#endif
