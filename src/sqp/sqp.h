#pragma once
#include <vector>
#include <string>
#include <gurobi_c++.h>
#include "sqp_fwd.h"
using std::vector;
using std::string;

const char* getGRBStatusString(int status);
void initializeGRB();

class ConvexPart {
	// constraint or cost that can be added to problem
public:
	void addToModel(GRBModel* model);
	void removeFromModel();

	ConvexPart() : m_inModel(false) {}
	virtual ~ConvexPart();
  bool m_inModel;
  GRBModel* m_model;
	
	vector<GRBVar> m_vars;

	vector<string> m_cntNames; // future names for constraints
	vector<GRBLinExpr> m_exprs; // expressions that are <= 0
	vector<GRBConstr> m_cnts; // expressions get turned into constraints

  vector<string> m_qcntNames; // future names for constraints
	vector<GRBQuadExpr> m_qexprs; // expression that are >= 0
	vector<GRBConstr> m_qcnts; // expressions get turned into constraints

};

class ConvexObjective : public ConvexPart {
public:
	GRBQuadExpr m_objective;
};

class ConvexConstraint : public ConvexPart {
public:
};


class Cost {
public:
	virtual ConvexObjectivePtr convexify()=0;
	virtual double evaluate() = 0;
	virtual string getName() {
	  return "unnamed cost";
	}

	GRBModel* m_model;
};

class Constraint {
public:
	virtual ConvexConstraintPtr convexify()=0;

	GRBModel* m_model;
};


class TrustRegion : public Constraint {
public:
	double m_shrinkage;
	TrustRegion() : m_shrinkage(1) {}
	virtual void adjustTrustRegion(double ratio) = 0;
};

class OptimizationProblem {
public:

	enum OptStatus {
		CONVERGED,
		ITERATION_LIMIT,
		SHRINKAGE_LIMIT,
		GRB_FAIL
	};
	
	vector<CostPtr> m_costs;
	vector<ConstraintPtr> m_cnts;	
	TrustRegionPtr m_tra;
	GRBModel* m_model;
	
	OptimizationProblem();
	virtual ~OptimizationProblem();
	
	virtual void updateValues() = 0;
	virtual void storeValues() = 0;
	virtual void rollbackValues() = 0;

	virtual void preOptimize() {} // do plots and stuff here
	virtual void postOptimize() {} // ditto
	OptStatus optimize();
  void addCost(CostPtr cost);
  void addConstraint(ConstraintPtr cnt);
  void setTrustRegion(TrustRegionPtr tra);

private:

  int convexOptimize();
	double getApproxObjective();
	vector<ConvexObjectivePtr> convexifyObjectives();
	vector<ConvexConstraintPtr> convexifyConstraints();
	vector<double> evaluateObjectives();
	void printObjectiveInfo(const vector<double>& oldExact,
	     const vector<double>& newApprox, const vector<double>& newExact);
	void setupConvexProblem(const vector<ConvexObjectivePtr>&, const vector<ConvexConstraintPtr>&);

};

