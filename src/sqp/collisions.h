#pragma once
#include <btBulletCollisionCommon.h>
#include "simulation/simulation_fwd.h"
#include "sqp/sqp_fwd.h"
#include "utils_sqp.h"
#include "simulation/openrave_fwd.h"
#include <Eigen/Dense>


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

/** Collision information for a link */
struct LinkCollision {
  double dist;
  int linkInd;
  btVector3 point; // world position
  btVector3 normal; // world normal
  float frac; // only used in continuous collision detection
  LinkCollision(double dist_, int linkInd_, const btVector3& point_, const btVector3& normal_):
    dist(dist_), linkInd(linkInd_), point(point_), normal(normal_) {}
};

/** A vector of collisions for links in one timestep */
typedef std::vector<LinkCollision> CartCollInfo;

/** A vector of vectors of collisions for each step in a trajectory */
typedef std::vector<CartCollInfo> TrajCartCollInfo;
struct JointCollInfo {
  std::vector<Eigen::VectorXd> jacs;
  std::vector<double> dists;
};
typedef std::vector< JointCollInfo > TrajJointCollInfo;



TrajCartCollInfo collectTrajCollisions(const Eigen::MatrixXd& traj, RaveRobotObject* rro, const std::vector<int>& dofInds, bool useAffine);
TrajCartCollInfo continuousTrajCollisions(const Eigen::MatrixXd& traj, OpenRAVE::RobotBasePtr robot, const std::vector<int>& dofInds, float allowedPen);
TrajJointCollInfo trajCartToJointCollInfo(const TrajCartCollInfo& in, const Eigen::MatrixXd& traj, RobotBasePtr robot,
    const std::vector<int>& dofInds, bool useAffine);
void countCollisions(const TrajJointCollInfo& trajCollInfo, double safeDist, int& nNear, int& nUnsafe, int& nColl);
