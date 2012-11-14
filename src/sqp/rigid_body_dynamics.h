#pragma once
#include "sqp.h"
#include <btBulletDynamicsCommon.h>
#include <Eigen/Dense>
#include <gurobi_c++.h>
#include "utils_sqp.h"

using namespace Eigen;

struct RigidBody {

  // set in ctor
  btRigidBody* m_rb;
  string m_name;

  Vector3d m_x0;
  Vector4d m_q0;
  Vector3d m_v0, m_w0;

  Vector3d m_x1_val;
  Vector4d m_q1_val;
  Vector3d m_v1_val, m_w1_val;

  // set in addToModel
  VarVector m_x1_var, m_r1_var, m_v1_var, m_w1_var;

  // backup
  Vector3d m_x1_backup;
  Vector4d m_q1_backup;
  Vector3d m_v1_backup, m_w1_backup;



  RigidBody(btRigidBody*, const string& name);

  void addToModel(GRBModel* model);
  void removeFromModel(GRBModel* model);

  void backup();
  void restore();
  void updateValues();
  void advanceTime();
  double getMass();
  void updateBullet();

  struct IntegrationConstraint : Constraint {
    RigidBody* m_rb;
    IntegrationConstraint(RigidBody*);
    ConvexConstraintPtr convexify(GRBModel* model);
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Contact {
  // set in ctor
  RigidBodyPtr m_bodyA, m_bodyB;
  VectorXd m_pt, m_normal;
  ExprVector m_vtan;

  // set in addToModel
  GRBVar m_fn;
  double m_fn_val;
  ExprVector m_fc;
  Vector3d m_fc_val;

  Contact(RigidBodyPtr bodyA, RigidBodyPtr bodyB, const VectorXd& pt, const VectorXd& normal);

  void addToModel(GRBModel* model);
  void removeFromModel(GRBModel* model);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Point2PointConstraint {
  Vector3d m_bodyA, m_bodyB;
  Vector3d m_ptA, m_ptB;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct PoseConstraint : Constraint {
  RigidBodyPtr m_body;
  Vector3d m_x;
  Vector4d m_q;
  void setPose(const Vector3d& x, const Vector4d& q);
  PoseConstraint(RigidBodyPtr body);
  ConvexConstraintPtr convexify(GRBModel* model);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct DynSolver : public Optimizer {

  vector<RigidBodyPtr> m_bodies;
  vector<ContactPtr> m_contacts;
  btCollisionWorld* m_world;

  DynSolver(btCollisionWorld* world);

	void updateValues();
	void storeValues();
	void rollbackValues();
	
  void addObject(RigidBodyPtr);
  
  void updateBulletWorld();  
  void updateContacts();
  void finishStep();
  
};

struct DynErrCost : public Cost {
  double m_forceErrCoeff;
  RigidBodyPtr m_body;
  DynErrCost(RigidBodyPtr);
	ConvexObjectivePtr convexify(GRBModel* model);
	double evaluate();
  string getName() {return "DynErrCost";}  
};

class DynTrustRegion : public TrustRegion {
public:
  DynSolver* m_solver;
  Vector3d m_x1_var, m_r1_var, m_v1_var, m_w1_var;
  DynTrustRegion(DynSolver* solver);
  ConvexConstraintPtr convexify(GRBModel* model);
  void adjustTrustRegion(double ratio);
};

typedef boost::shared_ptr<RigidBody> RigidBodyPtr;
typedef boost::shared_ptr<Contact> ContactPtr;

