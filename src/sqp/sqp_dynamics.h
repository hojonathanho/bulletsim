#pragma once

#include <vector>
#include <map>
#include <Eigen/Dense>
#include "utils_sqp.h"
#include "sqp.h"
#include <btBulletCollisionCommon.h>
using std::vector;
using std::map;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class DynamicsSolver : public OptimizationProblem {
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

  DynamicsSolver(btCollisionWorld* m_world, int numTimeSteps);

	double sumObjectives(vector<ConvexObjectivePtr>&);
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

class DynamicsCost : public Cost {
public:
  DynamicsSolver* m_solver;
  DynamicsSolver* getSolver();

};

class NoFricDynCost : public DynamicsCost {
public:
	vector<btRigidBody*> m_bodies;
	ConvexObjectivePtr convexify();
	double evaluate();
};

class OverlapCost : public DynamicsCost {
public:
	vector<btRigidBody*> m_bodies;
	ConvexObjectivePtr convexify();
	double evaluate();
};

StateSetterPtr makeStateSetter(vector<btRigidBody*>&);
