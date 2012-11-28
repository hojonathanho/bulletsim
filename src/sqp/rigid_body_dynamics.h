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

  Vector3d m_externalImpulse;

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
  void updateBullet0();
  void updateBullet1();
  void advanceTime();
  void predictMotion();
  double getMass();
  void updateBullet();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Contact {
  // set in ctor
  bool m_inModel;
  RigidBody* m_bodyA;
  RigidBody* m_bodyB;
  Vector3d m_worldA, m_worldB, m_localA, m_localB, m_normalB2A;
  double m_dist;

  // set in addToModel
  GRBVar m_fn;
  double m_fn_val;
  double m_fn_backup;
  VarVector m_ffr;
  Vector3d m_ffr_val;
  Vector3d m_ffr_backup;

  // set later
  GRBLinExpr m_distExpr;

  Contact(RigidBody* bodyA, RigidBody* bodyB, const btVector3& worldA, const btVector3& worldB,
  		const btVector3& localA, const btVector3& localB, const btVector3& normalB2A, double dist);
  ~Contact();
  void setData(const btVector3& worldA, const btVector3& worldB, const btVector3& normalB2A,
  		const btVector3& localA, const btVector3& localB, double dist);
  void addToModel(GRBModel* model);
  void removeFromModel(GRBModel* model);
  string getName();
  void updateValues();
  void backup();
  void restore();
  void calcDistExpr();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
struct SignedContact {
  Contact* contact;
  int sign;
  SignedContact(Contact* contact_, int sign_) : contact(contact_), sign(sign_) {}
};

void plotContacts(const vector<ContactPtr>& contacts);



struct DynSolver : public Optimizer {

  vector<RigidBodyPtr> m_bodies;
  vector<ContactPtr> m_contacts;
  vector<ContactPtr> m_contacts_backup;
  btCollisionWorld* m_world;
  vector<ContactPtr> m_toAdd, m_toRemove;

  DynSolver(btCollisionWorld* world);

  void step();

	void updateValues();
	void storeValues();
	void rollbackValues();
	
  void addObject(RigidBodyPtr);
  void postOptimize();
  void fixVariables();

  void updateContacts();
  void updateContactsFull();
  void finishStep();
  
  vector<SignedContact> getSignedContacts(RigidBody*);

};

struct IntegrationConstraint : public Constraint {
  RigidBodyPtr m_rb;
  IntegrationConstraint(RigidBodyPtr);
  ConvexConstraintPtr convexify(GRBModel* model);
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

struct ForcePenalty : public Cost {
  DynSolver* m_solver;
  double m_coeff;
  ForcePenalty(DynSolver*);
  double evaluate();
  ConvexObjectivePtr convexify(GRBModel* model);
  string getName() {return "ForcePenalty";}
};

struct ComplementarityCost : public Cost {
  DynSolver* m_solver;
  double m_coeff;
  ComplementarityCost(DynSolver*);
  double evaluate();
  ConvexObjectivePtr convexify(GRBModel* model);
  string getName() {return "Complementarity";}
};

struct ComplementarityCost2 : public Cost {
  DynSolver* m_solver;
  double m_coeff;
  ComplementarityCost2(DynSolver*);
  double evaluate();
  ConvexObjectivePtr convexify(GRBModel* model);
  string getName() {return "Complementarity2";}
};

struct FricCost : public Cost {
  DynSolver* m_solver;
  double m_coeff;
  FricCost(DynSolver*);
  double evaluate();
  ConvexObjectivePtr convexify(GRBModel* model);
  string getName() {return "Friction";}
};


struct FrictionConstraint : public Constraint {
  DynSolver* m_solver;
  double m_mu2;
  FrictionConstraint(DynSolver*);
  ConvexConstraintPtr convexify(GRBModel* model);
};

struct OverlapPenalty : public Cost {
  double m_coeff;
  DynSolver* m_solver;
  OverlapPenalty(DynSolver*);
  ConvexObjectivePtr convexify(GRBModel* model);
  double evaluate();
  string getName() {return "Overlap";}
};

struct OverlapConstraint : public NonlinearConstraint {
  DynSolver* m_solver;
  OverlapConstraint(DynSolver*);
  ConvexConstraintPtr convexify(GRBModel* model);
  double evaluate();
};


struct TangentMotion : public Cost {
  double m_coeff;
  DynSolver* m_solver;
  TangentMotion(DynSolver*);
  double evaluate();
  ConvexObjectivePtr convexify(GRBModel* model);
  string getName() {return "TangentVel";}
};

struct DynErrCost : public Cost {
  double m_forceErrCoeff;
  RigidBodyPtr m_body;
  DynSolver* m_solver;
  DynErrCost(RigidBodyPtr, DynSolver*);
	ConvexObjectivePtr convexify(GRBModel* model);
	double evaluate();
  string getName() {return "DynErr_"+m_body->m_name;}
};


class DynTrustRegion : public TrustRegion {
public:
  DynSolver* m_solver;
  double m_coeff;
  DynTrustRegion(DynSolver* solver);
  ConvexObjectivePtr convexObjective(GRBModel* model);
  ConvexConstraintPtr convexConstraint(GRBModel* model);
  void adjustTrustRegion(double ratio);
};

typedef boost::shared_ptr<RigidBody> RigidBodyPtr;
typedef boost::shared_ptr<Contact> ContactPtr;

