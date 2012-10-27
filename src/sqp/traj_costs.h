#pragma once
#include <btBulletDynamicsCommon.h>
#include <openrave/openrave.h>
#include "simulation/simulation_fwd.h"
#include "sqp/sqp_fwd.h"
#include "utils_sqp.h"
#include <Eigen/Dense>

#if 0
struct KinematicsInfo {
  OpenRAVE::RobotBasePtr robot;
  std::vector<OpenRAVE::KinBody::LinkPtr> links;
  std::vector<btRigidBody*> bodies;
  std::vector<OpenRAVE::KinBody::JointPtr> joints;
  std::vector<int> dofInds;
  std::vector<int> link2dofs;
};
KinematicsInfo getSingleArmKinematics(const OpenRAVE::RobotBase::ManipulatorPtr manip);
#endif

typedef std::pair<double,double> paird;


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


TrajCartCollInfo collectTrajCollisions(const Eigen::MatrixXd& traj, RaveRobotObject* robot, BulletRaveSyncher& brs, btCollisionWorld* world, const std::vector<int>& dofInds, bool useAffine);
TrajCartCollInfo continuousTrajCollisions(const Eigen::MatrixXd& traj, OpenRAVE::RobotBasePtr robot, BulletRaveSyncher& brs, btCollisionWorld* world, const std::vector<int>& dofInds, float allowedPen);
TrajJointCollInfo trajCartToJointCollInfo(const TrajCartCollInfo& in, const Eigen::MatrixXd& traj, OpenRAVE::RobotBasePtr robot,
    const std::vector<int>& dofInds, bool useAffine);

void countCollisions(const TrajJointCollInfo& trajCollInfo, double safeDist, int& nNear, int& nUnsafe, int& nColl);
