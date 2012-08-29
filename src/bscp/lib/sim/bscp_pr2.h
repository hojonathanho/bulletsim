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
      //getArmKinInfoFull(_pr2->robot, _manip->manip, _arm_links, _arm_joints, _chain_depth_of_bodies);
      getArmKinInfo(_pr2->robot,_arm_links, _arm_joints);
      for (int i = 0; i < _arm_links.size(); i++) {
      	  _bt_arm_links.push_back( _pr2->associatedObj(_arm_links[i])->rigidBody.get() );
      }
      _sync = new BulletRaveSyncer(_arm_links, _bt_arm_links);
 //     MatrixXd D; VectorXd d_bar;
 //     calc_contact_dist_approx(toEigenVectord(_pr2->getDOFValues(_active_dof_indices)), D, d_bar);
 //     cout << D << endl;
 //     cout << d_bar << endl;


/*
      vector<VectorXd> D_vec; vector<double> d_costs;
      vector<btRigidBody*> rigid_bodies(0);
      for (int i = 0; i < _arm_links.size(); i++) {
      	  rigid_bodies.push_back( _pr2->associatedObj(_arm_links[i])->rigidBody.get() );
      }
      calcMultiBodyJointSpaceCollisionInfo(rigid_bodies, world, _arm_joints, _chain_depth_of_bodies, D_vec, d_costs);

      for (int i = 0; i < D_vec.size(); i++) {

    	  cout << D_vec[i].transpose() << endl;
      }
*/


      M_set = false;
      N_set = false;

    }

    double x_upper_limit(const int i) { return _upper_joint_limits[i]; }
    double x_lower_limit(const int i) { return _lower_joint_limits[i]; }
    double u_upper_limit(const int i) { return  0.1; }
    double u_lower_limit(const int i) { return  -0.1; }

    vector<btRigidBody*> get_bullet_links() { return _bt_arm_links; }
    vector<KinBody::LinkPtr> get_OpenRAVE_links() { return _arm_links; }

/*
    void getArmKinInfoFull(const RobotBasePtr& robot, const RobotBase::ManipulatorPtr manip, std::vector<KinBody::LinkPtr>& armLinks, std::vector<KinBody::JointPtr>& armJoints, std::vector<int>& chainDepthOfBodies) {
      int rootLinkInd               = robot->GetLink("torso_lift_link")->GetIndex();
      BOOST_FOREACH(int ind, manip->GetArmIndices()) armJoints.push_back(robot->GetJointFromDOFIndex(ind));
      KinBody::JointPtr& firstJoint = armJoints[0];

      BOOST_FOREACH(KinBody::LinkPtr link, robot->GetLinks()) {
        vector<KinBody::JointPtr> jointChain;
        robot->GetChain(rootLinkInd, link->GetIndex(), jointChain);
        if (link->GetGeometries().size() && count(jointChain.begin(), jointChain.end(), firstJoint)) armLinks.push_back(link);
      }

      int nLinks                    = armLinks.size();

      chainDepthOfBodies            = vector<int>(nLinks,0);
      for (int iLink                = 0; iLink < armLinks.size(); ++iLink) {
        int linkInd                 = armLinks[iLink]->GetIndex();
        vector<KinBody::JointPtr> jointChain;
        robot->GetChain(rootLinkInd, armLinks[iLink]->GetIndex(), jointChain);
        BOOST_FOREACH(KinBody::JointPtr& joint0, armJoints) {
          BOOST_FOREACH(KinBody::JointPtr& joint1, jointChain) {
            if (joint0 == joint1) chainDepthOfBodies[iLink]++;
          }
        }
      }
    }
*/

/*
    void calcMultiBodyJointSpaceCollisionInfo(const std::vector<btRigidBody*>& bodies, btCollisionWorld* world, const vector<KinBody::JointPtr>& jointsInChain, const vector<int>& chainDepthOfBodies, std::vector<Eigen::VectorXd>& collJacs, std::vector<
        double>& collDists) {
      int nJoints = jointsInChain.size();

      for (int iBody = 0; iBody < bodies.size(); ++iBody) {
        std::vector<double> dists;
        std::vector<btVector3> normals, points;
        calcCollisionInfo(bodies[iBody], world, points, normals, dists);
        for (int iColl = 0; iColl < dists.size(); ++iColl) {
          VectorXd collJac(nJoints);
          for (int iJoint = 0; iJoint < chainDepthOfBodies[iBody]; ++iJoint) {
            const KinBody::JointPtr& joint = jointsInChain[iJoint];
            collJac(iJoint) = (points[iColl] - util::toBtVector(joint->GetAnchor())) .cross(util::toBtVector(jointsInChain[iJoint]->GetAxis())) .dot(normals[iColl]);
            // note: there's also an openrave function in KinBody to calculate jacobian that is more general
          }
          collJacs.push_back(collJac);
          collDists.push_back(dists[iColl]);
        }
      }
    }*/

    void p(const VectorXd& x_bar, VectorXd &p) {
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

	void dpdx_numerical(const VectorXd& x_bar, MatrixXd& D, VectorXd& d_bar) {
		int NX = x_bar.rows();
		D = MatrixXd::Zero(_arm_links.size(), NX);
		for (int i = 0; i < NX; i++) {
			VectorXd eps_vec = VectorXd::Zero(NX);
			eps_vec(i) = _eps;
			VectorXd x_pos = x_bar + eps_vec;
			VectorXd x_neg = x_bar - eps_vec;
			VectorXd px_pos(NX);
			VectorXd px_neg(NX);
			p(x_pos, px_pos);
			p(x_neg, px_neg);
			D.col(i) = (px_pos - px_neg) / (2 * _eps);
		}
		p(x_bar, d_bar);
		d_bar = d_bar - D * x_bar;
	}

	void dpdx(const VectorXd& x_bar, MatrixXd& D, VectorXd& d_bar) {
		dpdx_numerical(x_bar, D, d_bar);
	}

    void dpdx_analytical(const VectorXd& x_bar, MatrixXd& D, VectorXd& d_bar) {
    	//returns d(x) \approx Dx + d_bar; d_bar = d(x_bar) - Dx_bar;
    	vector<double> currentDOF = _pr2->getDOFValues(_active_dof_indices);
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

        _pr2->setDOFValues(_active_dof_indices, currentDOF);
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

  //void init(RaveRobotObject::Manipulator::Ptr, const std::vector<BulletObject::Ptr>&, Scene*, BulletRaveSyncher*, int decimation);
  //void setLength(int n);
  //void clear() {setLength(0);}
};




#endif
