#pragma once
#include <btBulletDynamicsCommon.h>
#include "functions.h"
#include <openrave/openrave.h>

void getArmKinInfo(const OpenRAVE::RobotBasePtr& robot, const OpenRAVE::RobotBase::ManipulatorPtr manip, std::vector<OpenRAVE::KinBody::LinkPtr>& armLinks, std::vector<OpenRAVE::KinBody::JointPtr>& armJoints, std::vector<int>& chainDepthOfBodies);

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

float calcCollisionCost(btRigidBody* body, btCollisionWorld* world);
void calcCollisionInfo(btRigidBody* body, btCollisionWorld* world, std::vector<btVector3>& pointsOnBody, std::vector<double>& dists);
void calcCollisionCostAndJointGrad(const std::vector<btRigidBody*>& bodies, btCollisionWorld* world, const OpenRAVE::RobotBasePtr& robot,
                                const std::vector<OpenRAVE::KinBody::JointPtr>& jointsInChain,
                                const std::vector<int>& chainDepthOfBodies,  /* i.e., how many joints in the kinematic chain precede each rigid body */
                                double& cost, Eigen::VectorXd& grad, int& nColl);


class BulletRaveSyncher {
public:
	std::vector<OpenRAVE::KinBody::LinkPtr> m_links; // what links are part of the object we're planning with
	std::vector<btRigidBody*> m_bodies; // what rigid bodies are part of the object we're planning with
	BulletRaveSyncher(const std::vector<OpenRAVE::KinBody::LinkPtr>& links, const std::vector<btRigidBody*>& bodies) :
		m_links(links), m_bodies(bodies) {}
	void updateBullet();
};

typedef std::pair< std::vector<Eigen::VectorXd>, std::vector<double> > TimestepCollisionInfo;
typedef std::vector< TimestepCollisionInfo > TrajCollisionInfo;

class CollisionCostEvaluator {
public:
  virtual float calcCost(const Eigen::MatrixXd& x)=0;
  virtual void calcCostAndGrad(const Eigen::MatrixXd& traj, double& val, Eigen::MatrixXd& deriv)=0;
  virtual TrajCollisionInfo collectCollisionInfo(const Eigen::MatrixXd& traj)=0;
};

class ArmCCE : public CollisionCostEvaluator {
public:
  typedef boost::shared_ptr<ArmCCE> Ptr;
  OpenRAVE::RobotBasePtr m_robot;
  btCollisionWorld* m_world;

  std::vector<OpenRAVE::KinBody::LinkPtr> m_links; // what links are part of the object we're planning with
  std::vector<btRigidBody*> m_bodies; // what rigid bodies are part of the object we're planning with
  std::vector<OpenRAVE::KinBody::JointPtr> m_joints; // active degrees of freedom in planning problem (currently assume all hinge joints)
  int m_nJoints; // number of joints
  std::vector<int> m_chainDepthOfBodies; // where does each link lie along the kinematic chain
  BulletRaveSyncher m_syncher;

  ArmCCE(OpenRAVE::RobotBasePtr robot, btCollisionWorld* world, const std::vector<OpenRAVE::KinBody::LinkPtr>& links,
		  const std::vector<btRigidBody*>& bodies, const std::vector<OpenRAVE::KinBody::JointPtr>& joints,
		  const std::vector<int>& depthOfBodies) :
	  m_robot(robot),
	  m_world(world),
	  m_links(links),
	  m_bodies(bodies),
	  m_joints(joints),
	  m_chainDepthOfBodies(depthOfBodies),
	  m_nJoints(joints.size()),
	  m_syncher(links, bodies)
  {}

  float calcCost(const Eigen::MatrixXd& x);
  void calcCostAndGrad(const Eigen::MatrixXd& traj, double& val, Eigen::MatrixXd& deriv);
  TrajCollisionInfo collectCollisionInfo(const Eigen::MatrixXd& traj);
  
};

void countCollisions(const TrajCollisionInfo& trajCollInfo, double safeDistMinusPadding, int& nNear, int& nUnsafe, int& nColl);


