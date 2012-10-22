#pragma once
#include <vector>
#include <string>
#include <gurobi_c++.h>
#include "sqp_fwd.h"
using std::vector;
using std::string;

const char* getGRBStatusString(int status);

class ConvexPart {
	// constraint or cost that can be added to problem
public:
	void addToModel();
	void removeFromModel();
	
	virtual ~ConvexPart() {}
	
	GRBModel* m_model;
	
	vector<GRBVar> m_vars;

	vector<string> m_cntNames; // future names for constraints
	vector<GRBLinExpr> m_exprs; // expressions that are <= 0
	vector<GRBConstr> m_cnts; // expressions get turned into constraints
};

class ConvexObjective : public ConvexPart {
public:
	GRBQuadExpr m_objective;
	double m_val; // value at convexification
};

class ConvexConstraint : public ConvexPart {
public:
};


class Cost {
public:
	virtual ConvexObjectivePtr convexify()=0;
	virtual double evaluate() = 0;

	GRBModel* m_model;
};

class Constraint {
public:
	virtual ConvexConstraintPtr convexify()=0;

	GRBModel* model;
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
	
	OptimizationProblem(GRBModel* model) : m_model(model) {}
	
	virtual void updateValues() = 0;
	virtual void storeValues() = 0;
	virtual void rollbackValues() = 0;
	virtual void preOptimize() {} // put plot callbacks and stuff here
	virtual void postOptimize() {} // ditto
	OptStatus optimize();
	int convexOptimize();
	double getApproxObjective();
	vector<ConvexObjectivePtr> convexifyObjectives();
	vector<ConvexConstraintPtr> convexifyConstraints();
	void addCost(CostPtr cost) {
		m_costs.push_back(cost);
	}
	void addConstraint(ConstraintPtr cnt) {
		m_cnts.push_back(cnt);
	}
	void setTrustRegion(TrustRegionPtr tra) {
		m_tra = tra;
		addConstraint(tra);
	}
	void setupConvexProblem(const vector<ConvexObjectivePtr>&, const vector<ConvexConstraintPtr>&);

};

