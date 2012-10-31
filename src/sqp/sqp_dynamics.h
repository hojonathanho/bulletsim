#pragma once

#include <vector>
#include <map>
#include <Eigen/Dense>
#include "utils_sqp.h"
#include "sqp.h"
#include <btBulletDynamicsCommon.h>
using std::vector;
using std::map;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class TrajDynSolver : public Optimizer {
public:


  const static int CW_TIME = -1; // timestep that is set up in collision world
  const static int POSE_DIM = 3; // dimensionality of pose

  vector<btRigidBody*> m_bodies;
  StateSetterPtr m_ss;
  map<btRigidBody*, MatrixXd>  m_obj2poses;
  map<btRigidBody*, VarArray> m_obj2poseVars;
  map<btRigidBody*, MatrixXd> m_obj2poses_backup;

  btCollisionWorld* m_world;
  int m_t; // timestep in current collision world
  int m_numTimesteps;
  bool m_initialized;

  TrajDynSolver(btCollisionWorld* m_world, int numTimeSteps);

	void updateValues();
	void storeValues();
	void rollbackValues();

  double fixPermanentVarsAndOptimize();
	
  VarVector getPoseVars(btRigidBody*, int t=CW_TIME);
  VectorXd getPoseValues(btRigidBody*, int t=CW_TIME);
  ExprVector getVelVars(btRigidBody*, int t=CW_TIME);
  VectorXd getVelValues(btRigidBody*, int t=CW_TIME);

  void setTimestep(int t);
  int getNumTimesteps();

  VectorXd getPose(btRigidBody*);

  void addObject(btRigidBody*);
  void addObject(btRigidBody* obj, const VectorXd&);
  void setStatic(btRigidBody* obj);
  void constrainPoses(btRigidBody*, const MatrixXd&);
  void initialize();

};

class TrajDynComponent {
public:
  TrajDynSolver* m_solver;
  TrajDynSolver* getSolver();

};


class TrajDynErrCost : public TrajDynComponent, public Cost {
public:
	vector<btRigidBody*> m_bodies;
	ConvexObjectivePtr convexify(GRBModel* model);
	double evaluate();
};

class TrajOverlapCost : public TrajDynComponent, public Cost {
public:
	vector<btRigidBody*> m_bodies;
	ConvexObjectivePtr convexify(GRBModel* model);
	double evaluate();
};

StateSetterPtr makeStateSetter(vector<btRigidBody*>&);


////////


class DynSolver : public Optimizer {
public:


  const static int POSE_DIM = 3; // dimensionality of pose

  vector<btRigidBody*> m_bodies;
  map<btRigidBody*, VectorXd>  m_obj2poses;
  map<btRigidBody*, VarVector> m_obj2poseVars;
  map<btRigidBody*, VectorXd> m_obj2poses_backup;
  map<btRigidBody*, VectorXd> m_obj2prevPoses;
  map<btRigidBody*, string> m_obj2name;
  map<btRigidBody*, vector<GRBConstr> > m_holdCnts;

  btCollisionWorld* m_world;
  bool m_initialized;

  DynSolver(btCollisionWorld* world);

	void updateValues();
	void storeValues();
	void rollbackValues();

  double fixPermanentVarsAndOptimize();
	
  VarVector& getPoseVars(btRigidBody*);
  VectorXd& getPoseValues(btRigidBody*);
  ExprVector getVelVars(btRigidBody*);

  void addObject(btRigidBody*, const string& name);
  void constrainPose(btRigidBody*);
  void getPosesFromWorld();
  void release(btRigidBody*);
  void initialize();

  void updateBodies();

};


class DynComponent {
public:
  DynComponent(DynSolver*);
  DynSolver* m_solver;
  DynSolver* getSolver() {
    return m_solver;
  }

};

class DynErrCost: public Cost, public DynComponent {
public:
  double m_forceErrCoeff, m_normalForceCoeff;
  DynErrCost(DynSolver* solver, double normalForceCoeff, double forceErrorCoeff);
	ConvexObjectivePtr convexify(GRBModel* model);
	double evaluate();
  string getName() {return "DynErrCost";}
};

class VelCost : public Cost, public DynComponent {
public:
   double m_coeff;
   VelCost(DynSolver* solver, double coeff);
   ConvexObjectivePtr convexify(GRBModel* model);
   double evaluate();
   string getName() {return "VelCost";}
 };


class DynOverlapCost: public Cost, public DynComponent {
public:
  double m_overlapCoeff;
  DynOverlapCost(DynSolver* solver, double overlapCoeff);
  ConvexObjectivePtr convexify(GRBModel* model);
  double evaluate();
  string getName() {return "DynOverlapCost";}
};

class DynTrustRegion : public TrustRegion, public DynComponent {
public:
  VectorXd m_maxDiffPerIter;
  DynTrustRegion(DynSolver* solver);
  ConvexConstraintPtr convexify(GRBModel* model);
  void adjustTrustRegion(double ratio);
};
