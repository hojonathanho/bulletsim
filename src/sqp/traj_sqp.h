#pragma once
#include "sqp.h"
#include "utils_sqp.h"
#include "simulation/simulation_fwd.h"
#include "sqp/sqp_fwd.h"
#include "simulation/openrave_fwd.h"

class btDynamicsWorld;
class TrajPlotter;
typedef boost::shared_ptr<TrajPlotter> TrajPlotterPtr;

class TrajOptimizer : public Optimizer {
public:
	VarArray m_vars;
	MatrixXd m_traj;
	MatrixXd m_traj_backup;
	VectorXd m_times;	
	vector<TrajPlotterPtr> m_plotters;
	bool m_initialized;
	
	TrajOptimizer();	
	void updateValues();
	void storeValues();
	void rollbackValues();
	void initialize(const MatrixXd& traj, const VectorXd& times);
	void preOptimize();
	void postOptimize();
	void setStartFixed();
	void setEndFixed();
	
};

class TrajComponent {
public:
	virtual void resample(const vector<double>& insertTimes, const VectorXd& oldTimes, const VectorXd& newTimes) {
		assert(0);
	}
	
	TrajComponent(TrajOptimizer*);
	TrajOptimizer* m_opt;

	MatrixXd& getTraj() {
		return m_opt->m_traj;
	}
	VarArray& getVars() {
		return m_opt->m_vars;
	}
	int getDof() {
	  return m_opt->m_traj.cols();
	}
	int getLength() {
		return m_opt->m_traj.rows();
	}
	VectorXd& getTimes() {
		return m_opt->m_times;
	}
};

class CollisionCost : public Cost, public TrajComponent {
public:
  RaveRobotObject* m_robot;
	vector<int> m_dofInds;
  bool m_useAffine;
  double m_coeff;
	
	CollisionCost(TrajOptimizer* opt, RaveRobotObject* robot, const vector<int>& dofInds, bool useAffine, double coeff);
	double evaluate();
  ConvexObjectivePtr convexify(GRBModel* model);	
	string getName() {return "CollisionCost";}
};

class CartPoseCost : public Cost, public TrajComponent {
public:
  RobotManipulatorPtr m_arm;
  void* m_link;
  vector<int> m_dofInds;
  bool m_useAffine;
  Vector3d m_posTarg;
  Vector4d m_rotTarg;
  double m_posCoeff, m_rotCoeff;
  bool m_l1;

  CartPoseCost(TrajOptimizer* opt, RobotManipulatorPtr arm, void* link, const vector<int>& dofInds,
               bool useAffine, const btTransform& goal, double posCoeff, double rotCoeff, bool l1);
	double evaluate();
  ConvexObjectivePtr convexify(GRBModel* model);	
	string getName() {return "CartPoseCost";}
};

class JntLenCost : public Cost, public TrajComponent {
public:
  double m_coeff;

  JntLenCost(TrajOptimizer* opt, double coeff);

	double evaluate();
  ConvexObjectivePtr convexify(GRBModel* model);	
	string getName() {return "JntLenCost";}
};
class JointBounds : public TrustRegion, public TrajComponent {
public:
  VectorXd m_maxDiffPerIter;
  VectorXd m_jointLowerLimit;
  VectorXd m_jointUpperLimit;

  JointBounds(TrajOptimizer* opt, const VectorXd& maxDiffPerIter, const VectorXd& jointLowerLimit, const VectorXd& jointUpperLimit);
  ConvexConstraintPtr convexify(GRBModel* model);
	void adjustTrustRegion(double ratio);
};

class CartVelCnt : public Constraint, public TrajComponent {
public:
  ConvexConstraintPtr convexify(GRBModel* model);
};
