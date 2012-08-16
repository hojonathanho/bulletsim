#include "traj_costs.h"
#include <simulation/util.h>
#include <boost/foreach.hpp>
#include "utils_scp.h"
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace Eigen;

void collisionInfo(btRigidBody* body, btCollisionWorld* world, std::vector<btVector3>& points, std::vector<btVector3>& normals, std::vector<float>& depths) {
  CollisionCollector collisionCollector;
  world->contactTest(body, collisionCollector);
  int nColl = collisionCollector.m_collisions.size();
  points.resize(nColl);
  normals.resize(nColl);
  depths.resize(nColl);
	for (int iColl = 0; iColl < nColl; ++iColl) {
		Collision& collision = collisionCollector.m_collisions[iColl];
		assert(collision.m_obj0 == body);
		points[iColl] = collision.m_world0;
		normals[iColl] = collision.m_normal;
		depths[iColl] = collision.m_distance;
	}
}

float collisionCost(btRigidBody* body, btCollisionWorld* world) {
  std::vector<btVector3> points, normals;
  std::vector<float> depths;
  collisionInfo(body, world, points, normals, depths);
  float cost = 0;
  for (int i = 0; i < depths.size(); ++i) {
    cost += depths[i]*depths[i];
  }
  return cost;
}


void collisionCostAndJointGrad(const std::vector<btRigidBody*>& bodies, btCollisionWorld* world, const RobotBasePtr& robot,
		const vector<KinBody::JointPtr>& jointsInChain, const vector<int>& chainDepthOfBodies,
		float& cost, Eigen::VectorXf& grad) {
  int nJoints = jointsInChain.size();
  
  cost = 0;
  grad = Eigen::VectorXf::Zero(nJoints);
  

  for (int iBody=0; iBody < bodies.size(); ++iBody) {
    std::vector<float> depths;
    std::vector<btVector3> normals, points;
    collisionInfo(bodies[iBody], world, points, normals, depths);

    for (int iColl = 0; iColl < depths.size(); ++iColl) {
    	for (int iJoint = 0; iJoint < chainDepthOfBodies[iBody]; ++iJoint) {
				const KinBody::JointPtr& joint = jointsInChain[iJoint];
				grad(iJoint) += (points[iColl] - toBtVector(joint->GetAnchor()))
						.cross(toBtVector(jointsInChain[iJoint]->GetAxis()))
						.dot(normals[iColl]);
        cost += depths[iColl]*depths[iColl];
      }
    }
  }
}


float CollisionCostEvaluator::calcVal(const Eigen::MatrixXf& traj) {
  float val = 0;
  int nSteps = x.size() / m_nJoints;

  vector<int> armInds;
  BOOST_FOREACH(KinBody::JointPtr joint, m_joints) armInds.push_back(joint->GetDOFIndex());
  m_robot->SetActiveDOFs(armInds);
  vector<double> curVals;
  m_robot->GetActiveDOFValues(curVals);

  for (int iStep=0; iStep < nSteps; ++iStep) {
	m_robot->SetActiveDOFValues(toDoubleVec(traj.row(iStep)));
	updateBullet();
    for (int iBody=0; iBody < m_bodies.size(); ++iBody) {
      val += collisionCost(m_bodies[iBody], m_world);
    }
  }  

  m_robot->SetActiveDOFValues(curVals);
  updateBullet();
  return val;  
}

void CollisionCostEvaluator::updateBullet() {
	for (int iBody=0; iBody < m_bodies.size(); ++iBody) {
		m_bodies[iBody]->setCenterOfMassTransform(util::toBtTransform(m_links[iBody]->GetTransform(), GeneralConfig::scale));
	}
}


void CollisionCostEvaluator::calcCostAndGrad(const Eigen::MatrixXf& traj, float& val, Eigen::MatrixXf& deriv) {
  int nSteps = traj.rows();

  vector<int> armInds;
  BOOST_FOREACH(KinBody::JointPtr joint, m_joints) armInds.push_back(joint->GetDOFIndex());
  m_robot->SetActiveDOFs(armInds);
  vector<double> curVals;
  m_robot->GetActiveDOFValues(curVals);

  grad = MatrixXf::Zero(nSteps, m_nJoints);

  for (int iStep=0; iStep < nSteps; ++iStep) {
	m_robot->SetActiveDOFValues(toDoubleVec(traj.row(iStep)));
	updateBullet();
    for (int iBody=0; iBody < m_bodies.size(); ++iBody) {
      VectorXf gradHere;
      float costHere;
      collisionCostAndJointGrad(m_bodies, m_world, m_robot, m_joints, m_chainDepthOfBodies, costHere, gradHere);
      grad.row(iStep) += gradHere;
      val += costHere;
    }
  }

  m_robot->SetActiveDOFValues(curVals);
  updateBullet();
  
}

